// SPDX-License-Identifier: Apache-2.0
// Independent TensorRT10/CUDA implementation of the Autoware base ONNX contract.
// Feature semantics and decoder conventions follow TIER IV/NVIDIA's Apache-2.0
// autoware_lidar_centerpoint, but no ROS or Autoware runtime library is required.
#include <NvInfer.h>
#include <NvInferVersion.h>
#include <NvOnnxParser.h>
#include <cuda_runtime.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#if NV_TENSORRT_MAJOR < 10
#error This backend requires TensorRT 10; rebuild on the target JetPack 6.2 Orin.
#endif

namespace {
constexpr int GRID = 480, MAX_VOXELS = 40000, POINTS_PER_VOXEL = 32;
constexpr int MAX_POINTS = 2000000, MAX_BOXES = 32768;
thread_local std::string error_message;

void check(cudaError_t status) {
  if (status != cudaSuccess) throw std::runtime_error(cudaGetErrorString(status));
}

class Logger : public nvinfer1::ILogger {
public:
  std::string last;
  void log(Severity severity, char const* message) noexcept override {
    if (severity <= Severity::kWARNING) last = message;
  }
};

std::vector<char> read_file(std::string const& path) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) throw std::runtime_error("Cannot read model/engine: " + path);
  auto size = input.tellg();
  if (size <= 0) throw std::runtime_error("Empty model/engine: " + path);
  std::vector<char> bytes(static_cast<size_t>(size));
  input.seekg(0); input.read(bytes.data(), bytes.size());
  if (!input) throw std::runtime_error("Incomplete model/engine: " + path);
  return bytes;
}

size_t volume(nvinfer1::Dims dims) {
  size_t total = 1;
  for (int i=0; i<dims.nbDims; ++i) {
    if (dims.d[i] <= 0) throw std::runtime_error("Unresolved TensorRT shape");
    total *= static_cast<size_t>(dims.d[i]);
  }
  return total;
}

struct Engine {
  Logger logger;
  std::unique_ptr<nvinfer1::IRuntime> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> engine;
  std::unique_ptr<nvinfer1::IExecutionContext> context;
  std::unordered_map<std::string,float*> tensors;
  std::unordered_map<std::string,size_t> capacities;
  std::string input_name;

  Engine(std::string const& onnx, std::string const& cache, bool encoder) {
    std::vector<char> bytes;
    if (std::filesystem::exists(cache)) bytes = read_file(cache);
    else {
      std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(logger));
      if (!builder) throw std::runtime_error("Cannot create TensorRT builder");
      std::unique_ptr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(0U));
      std::unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*network,logger));
      if (!parser || !parser->parseFromFile(onnx.c_str(),static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
        throw std::runtime_error("ONNX parse failed: "+logger.last);
      if (network->getNbInputs()!=1) throw std::runtime_error("Expected one ONNX input");
      input_name=network->getInput(0)->getName();
      if (input_name!=(encoder ? "input_features" : "spatial_features"))
        throw std::runtime_error("Wrong CenterPoint model input");
      std::unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
      config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,2ULL<<30);
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
      auto* profile=builder->createOptimizationProfile();
      nvinfer1::Dims lo=encoder ? nvinfer1::Dims3{1,32,9} : nvinfer1::Dims4{1,32,GRID,GRID};
      nvinfer1::Dims opt=encoder ? nvinfer1::Dims3{12000,32,9} : lo;
      nvinfer1::Dims hi=encoder ? nvinfer1::Dims3{MAX_VOXELS,32,9} : lo;
      if (!profile->setDimensions(input_name.c_str(),nvinfer1::OptProfileSelector::kMIN,lo)
          || !profile->setDimensions(input_name.c_str(),nvinfer1::OptProfileSelector::kOPT,opt)
          || !profile->setDimensions(input_name.c_str(),nvinfer1::OptProfileSelector::kMAX,hi)
          || config->addOptimizationProfile(profile)<0)
        throw std::runtime_error("Invalid CenterPoint TensorRT profile");
      std::unique_ptr<nvinfer1::IHostMemory> plan(builder->buildSerializedNetwork(*network,*config));
      if (!plan) throw std::runtime_error("TensorRT engine build failed: "+logger.last);
      auto const* data=static_cast<char const*>(plan->data());
      bytes.assign(data,data+plan->size());
      std::filesystem::create_directories(std::filesystem::path(cache).parent_path());
      std::string partial=cache+".part";
      std::ofstream output(partial,std::ios::binary);
      output.write(bytes.data(),bytes.size()); output.close();
      if (!output) throw std::runtime_error("Cannot write TensorRT cache");
      std::filesystem::rename(partial,cache);
    }
    runtime.reset(nvinfer1::createInferRuntime(logger));
    if (!runtime) throw std::runtime_error("Cannot create TensorRT runtime");
    engine.reset(runtime->deserializeCudaEngine(bytes.data(),bytes.size()));
    if (!engine) throw std::runtime_error("Engine incompatible/corrupt; remove cache and rebuild: "+logger.last);
    context.reset(engine->createExecutionContext());
    if (!context) throw std::runtime_error("Cannot create TensorRT context");
    input_name=encoder ? "input_features" : "spatial_features";
    nvinfer1::Dims maximum=encoder ? nvinfer1::Dims3{MAX_VOXELS,32,9} : nvinfer1::Dims4{1,32,GRID,GRID};
    if (!context->setInputShape(input_name.c_str(),maximum)) throw std::runtime_error("Wrong base model input shape");
    for (int i=0;i<engine->getNbIOTensors();++i) {
      std::string name=engine->getIOTensorName(i);
      if (engine->getTensorDataType(name.c_str())!=nvinfer1::DataType::kFLOAT)
        throw std::runtime_error("Base model I/O must be float32");
      size_t count=volume(context->getTensorShape(name.c_str()));
      float* ptr=nullptr; check(cudaMalloc(&ptr,count*sizeof(float)));
      tensors[name]=ptr; capacities[name]=count;
      if (!context->setTensorAddress(name.c_str(),ptr)) throw std::runtime_error("Cannot bind tensor "+name);
    }
    auto dims=context->getTensorShape(encoder ? "pillar_features" : "heatmap");
    if ((encoder && (volume(dims)!=static_cast<size_t>(MAX_VOXELS)*32))
        || (!encoder && (dims.nbDims!=4 || dims.d[1]!=5 || dims.d[2]!=GRID || dims.d[3]!=GRID)))
      throw std::runtime_error("This backend supports the base v4.1 model only");
  }
  ~Engine() { for(auto const& item:tensors) cudaFree(item.second); }
  float* at(std::string const& name) { return tensors.at(name); }
  void run(int pillars,cudaStream_t stream) {
    if (input_name=="input_features"
        && !context->setInputShape(input_name.c_str(),nvinfer1::Dims3{pillars,32,9}))
      throw std::runtime_error("Cannot set pillar count");
    if (!context->enqueueV3(stream)) throw std::runtime_error("TensorRT inference failed: "+logger.last);
  }
};

__global__ void voxelize(float const* points,int n,unsigned* counts,float* storage) {
  int i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i>=n)return;
  float x=points[i*4],y=points[i*4+1],z=points[i*4+2],t=points[i*4+3];
  if(!isfinite(x)||!isfinite(y)||!isfinite(z)||!isfinite(t)
      || x< -76.8f || x>=76.8f || y< -76.8f || y>=76.8f || z< -4.f || z>=6.f)return;
  int ix=min(GRID-1,max(0,static_cast<int>(floorf((x+76.8f)/.32f))));
  int iy=min(GRID-1,max(0,static_cast<int>(floorf((y+76.8f)/.32f))));
  int cell=iy*GRID+ix;
  unsigned slot=atomicAdd(counts+cell,1U);
  if(slot<32){float* dst=storage+(cell*32+slot)*4;dst[0]=x;dst[1]=y;dst[2]=z;dst[3]=t;}
}

__global__ void features(unsigned const* counts,float const* storage,unsigned* number,
                         int* cells,float* output) {
  int cell=blockIdx.x;
  unsigned count=min(counts[cell],32U);
  if(!count)return;
  __shared__ unsigned pillar;
  if(threadIdx.x==0)pillar=atomicAdd(number,1U);
  __syncthreads();
  if(pillar>=MAX_VOXELS)return;
  if(threadIdx.x==0)cells[pillar]=cell;
  int slot=threadIdx.x;
  float4 p=slot<count ? reinterpret_cast<float4 const*>(storage)[cell*32+slot] : make_float4(0,0,0,0);
  float sx=p.x,sy=p.y,sz=p.z;
  for(int offset=16;offset>0;offset>>=1){
    sx+=__shfl_down_sync(0xffffffff,sx,offset);
    sy+=__shfl_down_sync(0xffffffff,sy,offset);
    sz+=__shfl_down_sync(0xffffffff,sz,offset);
  }
  sx=__shfl_sync(0xffffffff,sx,0)/count;
  sy=__shfl_sync(0xffffffff,sy,0)/count;
  sz=__shfl_sync(0xffffffff,sz,0)/count;
  // ONNX input is genuinely [N,32,9], confirmed by its initial MatMul.
  float* dst=output+(pillar*32+slot)*9;
  if(slot<count){
    dst[0]=p.x;dst[1]=p.y;dst[2]=p.z;dst[3]=p.w;
    dst[4]=p.x-sx;dst[5]=p.y-sy;dst[6]=p.z-sz;
    dst[7]=p.x-(cell%GRID*.32f-76.8f+.16f);
    dst[8]=p.y-(cell/GRID*.32f-76.8f+.16f);
  }else{for(int k=0;k<9;++k)dst[k]=0.f;}
}

__global__ void scatter(float const* pillars,int const* cells,int n,float* spatial) {
  int i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i>=n*32)return;
  int pillar=i/32,channel=i%32;
  spatial[channel*GRID*GRID+cells[pillar]]=pillars[i];
}

__global__ void decode(float const* hm,float const* reg,float const* height,
                       float const* dim,float const* rot,float const* thresholds,
                       unsigned* number,unsigned* invalid,float* boxes) {
  int i=blockIdx.x*blockDim.x+threadIdx.x;
  constexpr int S=GRID*GRID;
  if(i>=S)return;
  int label=0;float score=-1.f;
  for(int c=0;c<5;++c){
    float value=hm[c*S+i];
    if(!isfinite(value)){atomicAdd(invalid,1U);return;}
    float v=1.f/(1.f+expf(-value));if(v>score){score=v;label=c;}
  }
  if(score<thresholds[label])return;
  float sn=rot[i],cs=rot[S+i];
  if(label!=4 && sqrtf(sn*sn+cs*cs)<.3f)return;
  float x=(i%GRID+reg[i])*.32f-76.8f,y=(i/GRID+reg[S+i])*.32f-76.8f;
  float w=expf(dim[i]),l=expf(dim[S+i]),h=expf(dim[2*S+i]);
  if(!isfinite(x)||!isfinite(y)||!isfinite(height[i])||!isfinite(l)||!isfinite(w)||!isfinite(h)
      ||!isfinite(sn)||!isfinite(cs)||l<=0.f||w<=0.f||h<=0.f){atomicAdd(invalid,1U);return;}
  unsigned k=atomicAdd(number,1U);
  if(k>=MAX_BOXES)return;
  float* out=boxes+k*9;
  out[0]=x;out[1]=y;out[2]=height[i];out[3]=l;out[4]=w;out[5]=h;
  out[6]=atan2f(sn,cs);out[7]=score;out[8]=static_cast<float>(label);
}

struct Detector {
  cudaStream_t stream=nullptr;
  std::unique_ptr<Engine> encoder,head;
  float *points=nullptr,*storage=nullptr,*boxes=nullptr,*thresholds=nullptr;
  unsigned *counts=nullptr,*number=nullptr,*box_number=nullptr,*invalid=nullptr;
  int* cells=nullptr;
  unsigned last_pillars=0;
  Detector(std::string const& models,std::string const& cache) {
    check(cudaStreamCreate(&stream));
    cudaDeviceProp prop{};check(cudaGetDeviceProperties(&prop,0));
    std::string key=".base-v4.1.sm"+std::to_string(prop.major)+std::to_string(prop.minor)
      +".trt"+std::to_string(NV_TENSORRT_MAJOR)+"."+std::to_string(NV_TENSORRT_MINOR)
      +"."+std::to_string(NV_TENSORRT_PATCH)+".fp16.engine";
    encoder=std::make_unique<Engine>(models+"/pts_voxel_encoder.onnx",cache+"/encoder"+key,true);
    head=std::make_unique<Engine>(models+"/pts_backbone_neck_head.onnx",cache+"/head"+key,false);
    check(cudaMalloc(&points,static_cast<size_t>(MAX_POINTS)*4*sizeof(float)));
    check(cudaMalloc(&storage,static_cast<size_t>(GRID)*GRID*32*4*sizeof(float)));
    check(cudaMalloc(&counts,GRID*GRID*sizeof(unsigned)));
    check(cudaMalloc(&number,sizeof(unsigned)));
    check(cudaMalloc(&cells,MAX_VOXELS*sizeof(int)));
    check(cudaMalloc(&boxes,MAX_BOXES*9*sizeof(float)));
    check(cudaMalloc(&box_number,sizeof(unsigned)));
    check(cudaMalloc(&invalid,sizeof(unsigned)));
    check(cudaMalloc(&thresholds,5*sizeof(float)));
  }
  ~Detector(){
    if(stream)cudaStreamSynchronize(stream);
    for(void* p:{static_cast<void*>(points),static_cast<void*>(storage),static_cast<void*>(boxes),
                static_cast<void*>(thresholds),static_cast<void*>(counts),static_cast<void*>(number),
                static_cast<void*>(box_number),static_cast<void*>(invalid),static_cast<void*>(cells)})if(p)cudaFree(p);
    encoder.reset();head.reset();
    if(stream)cudaStreamDestroy(stream);
  }
  int run(float const* input,int n,float const* scores,float* output,int capacity) {
    if(n<=0 || n>MAX_POINTS)throw std::runtime_error("Invalid/oversized point buffer");
    check(cudaMemcpyAsync(points,input,static_cast<size_t>(n)*4*sizeof(float),cudaMemcpyHostToDevice,stream));
    check(cudaMemsetAsync(counts,0,GRID*GRID*sizeof(unsigned),stream));
    check(cudaMemsetAsync(number,0,sizeof(unsigned),stream));
    voxelize<<<(n+255)/256,256,0,stream>>>(points,n,counts,storage);
    features<<<GRID*GRID,32,0,stream>>>(counts,storage,number,cells,encoder->at("input_features"));
    check(cudaGetLastError());
    check(cudaMemcpyAsync(&last_pillars,number,sizeof(unsigned),cudaMemcpyDeviceToHost,stream));
    check(cudaStreamSynchronize(stream));
    if(last_pillars>MAX_VOXELS)throw std::runtime_error("Base model max_voxel_size exceeded");
    if(!last_pillars)return 0;
    encoder->run(last_pillars,stream);
    check(cudaMemsetAsync(head->at("spatial_features"),0,32ULL*GRID*GRID*sizeof(float),stream));
    scatter<<<(last_pillars*32+255)/256,256,0,stream>>>(encoder->at("pillar_features"),cells,last_pillars,head->at("spatial_features"));
    head->run(1,stream);
    check(cudaMemsetAsync(box_number,0,sizeof(unsigned),stream));
    check(cudaMemsetAsync(invalid,0,sizeof(unsigned),stream));
    check(cudaMemcpyAsync(thresholds,scores,5*sizeof(float),cudaMemcpyHostToDevice,stream));
    decode<<<(GRID*GRID+255)/256,256,0,stream>>>(head->at("heatmap"),head->at("reg"),
      head->at("height"),head->at("dim"),head->at("rot"),thresholds,box_number,invalid,boxes);
    check(cudaGetLastError());
    unsigned count=0,bad=0;
    check(cudaMemcpyAsync(&count,box_number,sizeof(unsigned),cudaMemcpyDeviceToHost,stream));
    check(cudaMemcpyAsync(&bad,invalid,sizeof(unsigned),cudaMemcpyDeviceToHost,stream));
    check(cudaStreamSynchronize(stream));
    if(bad)throw std::runtime_error("Non-finite/invalid neural head output");
    if(count>MAX_BOXES || count>static_cast<unsigned>(capacity))
      throw std::runtime_error("Neural decoder candidate capacity exceeded");
    check(cudaMemcpyAsync(output,boxes,static_cast<size_t>(count)*9*sizeof(float),cudaMemcpyDeviceToHost,stream));
    check(cudaStreamSynchronize(stream));
    return static_cast<int>(count);
  }
};
} // namespace

extern "C" {
int campus_cp_api_version(){return 1;}
char const* campus_cp_last_error(){return error_message.c_str();}
void* campus_cp_create(char const* models,char const* cache){
  try{error_message.clear();return new Detector(models,cache);}
  catch(std::exception const& e){error_message=e.what();return nullptr;}
}
int campus_cp_run(void* handle,float const* points,int count,float const* scores,float* boxes,int capacity){
  try{
    if(!handle)throw std::runtime_error("Null CenterPoint handle");
    return static_cast<Detector*>(handle)->run(points,count,scores,boxes,capacity);
  }catch(std::exception const& e){error_message=e.what();return -1;}
}
int campus_cp_pillar_count(void* handle){return handle ? static_cast<Detector*>(handle)->last_pillars : 0;}
int campus_cp_export_features(void* handle,float* output,int* cell_output,int capacity){
  try{
    if(!handle)throw std::runtime_error("Null CenterPoint handle");
    auto* detector=static_cast<Detector*>(handle);
    auto count=detector->last_pillars;
    if(count>static_cast<unsigned>(capacity))throw std::runtime_error("Feature export buffer too small");
    check(cudaMemcpy(output,detector->encoder->at("input_features"),static_cast<size_t>(count)*32*9*sizeof(float),cudaMemcpyDeviceToHost));
    check(cudaMemcpy(cell_output,detector->cells,count*sizeof(int),cudaMemcpyDeviceToHost));
    return count;
  }catch(std::exception const& e){error_message=e.what();return -1;}
}
void campus_cp_destroy(void* handle){delete static_cast<Detector*>(handle);}
}

import json,hashlib,zipfile,shutil,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent;CLONE=Path('/tmp/plan2-rviz-upload-20261107')
manifest=json.loads((OUT/'modified_files.json').read_text())['production_files']
backups={};new=[]
for name,expected in manifest.items():
 source=ROOT/name
 assert hashlib.sha256(source.read_bytes()).hexdigest()==expected,name
 old=None
 for base in (ROOT/'diagnostics/dynamic_yield_20261108/before',OUT/'before',CLONE):
  candidate=base/name
  if candidate.exists():old=candidate;break
 if old is None:
  assert source.name in ('perception_core.py','pose_velocity.py'),name
  new.append(name)
 else:
  dest=OUT/'rollback'/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(old,dest)
  backups[name]=hashlib.sha256(dest.read_bytes()).hexdigest()
update_zip=OUT/'car_gpt_modified_20261108.zip';rollback_zip=OUT/'car_gpt_rollback_20261108.zip'
with zipfile.ZipFile(update_zip,'w',zipfile.ZIP_DEFLATED) as z:
 for name in manifest:z.write(ROOT/name,name)
with zipfile.ZipFile(rollback_zip,'w',zipfile.ZIP_DEFLATED) as z:
 for name in backups:z.write(OUT/'rollback'/name,name)
for path in (update_zip,rollback_zip):
 with zipfile.ZipFile(path) as z:assert z.testzip() is None
(OUT/'rollback_manifest.json').write_text(json.dumps({'existing_files':backups,'new_files_to_remove_after_restoring':new},indent=2)+'\n')
# Copy only the explicit production-file manifest to the authorized patch repo.
for name in manifest:
 dest=CLONE/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,dest)
subprocess.run(['git','add','--',*manifest],cwd=CLONE,check=True)
staged=subprocess.check_output(['git','diff','--cached','--name-only'],cwd=CLONE,text=True).splitlines()
assert set(staged)==set(manifest),(staged,list(manifest))
(OUT/'upload_manifest.json').write_text(json.dumps({'files':staged,'zip':str(update_zip),'rollback_zip':str(rollback_zip)},indent=2)+'\n')
print(json.dumps({'production_files':len(staged),'modified_zip_bytes':update_zip.stat().st_size,'rollback_files':len(backups),'new_files':new},indent=2))

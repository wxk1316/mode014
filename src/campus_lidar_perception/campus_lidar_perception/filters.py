"""Linear CV and genuine interacting CV/CA Kalman models in Cartesian coordinates.

IMM mixes conditional states/covariances before prediction, updates each model
with its innovation likelihood, and includes between-model variance in fusion.
"""
import math
import numpy as np


def _correct(mean, covariance, measurement, sigma):
    h=np.eye(2,len(mean))
    r=np.eye(2)*sigma**2
    innovation=np.asarray(measurement,float)-mean[:2]
    s=h@covariance@h.T+r
    sign,logdet=np.linalg.slogdet(s)
    if sign<=0 or not np.isfinite(s).all():
        raise ValueError('invalid innovation covariance')
    k=np.linalg.solve(s,h@covariance).T
    posterior=mean+k@innovation
    a=np.eye(len(mean))-k@h
    p=a@covariance@a.T+k@r@k.T
    p=.5*(p+p.T)
    likelihood=-.5*(2*math.log(2*math.pi)+logdet+innovation@np.linalg.solve(s,innovation))
    return posterior,p,float(likelihood)


class CVKalman:
    name='cv'
    def __init__(self, state, covariance, acceleration_variance=2.):
        self.state=np.asarray(state,float).copy()
        self.covariance=np.asarray(covariance,float).copy()
        self.acceleration_variance=acceleration_variance

    def predict(self,dt):
        if not math.isfinite(dt) or dt<0:raise ValueError('invalid filter dt')
        f=np.eye(4);f[0,2]=f[1,3]=dt
        g=np.array([[dt*dt/2,0],[0,dt*dt/2],[dt,0],[0,dt]])
        self.state=f@self.state
        self.covariance=f@self.covariance@f.T+self.acceleration_variance*g@g.T

    def update(self,measurement,sigma):
        self.state,self.covariance,_=_correct(self.state,self.covariance,measurement,sigma)

    def limit_speed(self,maximum):
        speed=np.linalg.norm(self.state[2:4])
        if speed>maximum:self.state[2:4]*=maximum/speed

    def limit_displacement(self,previous,maximum):
        delta=self.state[:2]-previous;distance=np.linalg.norm(delta)
        if distance>maximum:self.state[:2]=previous+delta*maximum/distance

    @property
    def probabilities(self):return [1.]
    @property
    def acceleration(self):return [0.,0.]


class IMMKalman:
    name='imm'
    def __init__(self,state,covariance,acceleration_variance=2.,jerk_variance=6.,
                 acceleration_prior_variance=4.,stay_probability=.95,cv_probability=.7):
        if not 0<stay_probability<1 or not 0<cv_probability<1:
            raise ValueError('IMM probabilities must be between zero and one')
        if min(acceleration_variance,jerk_variance,acceleration_prior_variance)<=0:
            raise ValueError('IMM variances must be positive')
        mean=np.zeros(6);mean[:4]=np.asarray(state,float)
        p=np.eye(6)*acceleration_prior_variance;p[:4,:4]=covariance
        self.means=np.stack((mean.copy(),mean.copy()))
        self.covariances=np.stack((p.copy(),p.copy()))
        self.mu=np.array([cv_probability,1-cv_probability])
        self.transition=np.array([[stay_probability,1-stay_probability],
                                  [1-stay_probability,stay_probability]])
        self.acceleration_variance=acceleration_variance
        self.jerk_variance=jerk_variance
        self.acceleration_prior_variance=acceleration_prior_variance
        self._fuse()

    def _fuse(self):
        self.mean=np.einsum('i,ij->j',self.mu,self.means)
        delta=self.means-self.mean
        self.p=np.einsum('i,ijk->jk',self.mu,self.covariances)+np.einsum('i,ij,ik->jk',self.mu,delta,delta)
        self.p=.5*(self.p+self.p.T)

    def _mix(self):
        # transition[i,j] is probability of source model i -> destination j.
        prior=self.mu@self.transition
        weights=self.mu[:,None]*self.transition/prior[None,:]
        means=np.einsum('ij,ik->jk',weights,self.means)
        covariances=np.zeros_like(self.covariances)
        for j in range(2):
            delta=self.means-means[j]
            covariances[j]=np.einsum('i,ijk->jk',weights[:,j],self.covariances)
            covariances[j]+=np.einsum('i,ij,ik->jk',weights[:,j],delta,delta)
        self.means,self.covariances,self.mu=means,covariances,prior

    def predict(self,dt):
        if not math.isfinite(dt) or dt<0:raise ValueError('invalid filter dt')
        self._mix()
        cv=np.eye(6);cv[0,2]=cv[1,3]=dt;cv[4:,:]=0.
        g=np.zeros((6,2));g[0,0]=g[1,1]=dt*dt/2;g[2,0]=g[3,1]=dt
        q_cv=self.acceleration_variance*g@g.T
        # The CV model embeds nonpersistent acceleration uncertainty for mixing.
        q_cv[4,4]=q_cv[5,5]=self.acceleration_prior_variance
        ca=np.eye(6);ca[0,2]=ca[1,3]=ca[2,4]=ca[3,5]=dt
        ca[0,4]=ca[1,5]=dt*dt/2
        q_ca=np.zeros((6,6))
        one=self.jerk_variance*np.array([[dt**5/20,dt**4/8,dt**3/6],
                                         [dt**4/8,dt**3/3,dt**2/2],
                                         [dt**3/6,dt**2/2,dt]])
        for indices in ([0,2,4],[1,3,5]):q_ca[np.ix_(indices,indices)]=one
        for i,(f,q) in enumerate(((cv,q_cv),(ca,q_ca))):
            self.means[i]=f@self.means[i]
            self.covariances[i]=f@self.covariances[i]@f.T+q
        self._fuse()

    def update(self,measurement,sigma):
        log_weights=np.log(np.maximum(self.mu,1e-300))
        for i in range(2):
            self.means[i],self.covariances[i],likelihood=_correct(self.means[i],self.covariances[i],measurement,sigma)
            log_weights[i]+=likelihood
        weights=np.exp(log_weights-log_weights.max())
        self.mu=weights/weights.sum()
        self._fuse()

    def limit_speed(self,maximum):
        for mean in self.means:
            speed=np.linalg.norm(mean[2:4])
            if speed>maximum:mean[2:4]*=maximum/speed
        self._fuse()

    def limit_displacement(self,previous,maximum):
        delta=self.state[:2]-previous;distance=np.linalg.norm(delta)
        if distance>maximum:
            # A common translation preserves between-model offsets and covariance.
            correction=previous+delta*maximum/distance-self.state[:2]
            self.means[:,:2]+=correction
            self._fuse()

    @property
    def state(self):return self.mean[:4]
    @property
    def covariance(self):return self.p[:4,:4]
    @property
    def probabilities(self):return self.mu.tolist()
    @property
    def acceleration(self):return self.mean[4:6].tolist()

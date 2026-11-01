"""Host tests: independent 70-digit EKF reference, four inputs, failures, aliases."""
import ctypes as c
from decimal import Decimal as D, localcontext
import json
import math
import os
from pathlib import Path
import random
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
V2 = c.c_double * 2
V4 = c.c_double * 4
M2 = V2 * 2
M4 = V4 * 4

def matrix(kind, values):
    return kind(*(kind._type_(*row) for row in values))

def transpose(a):
    return list(map(list, zip(*a)))

def multiply(a, b):
    return [[sum(x*y for x,y in zip(row,col)) for col in zip(*b)] for row in a]

def plus(a,b):
    return [[x+y for x,y in zip(ar,br)] for ar,br in zip(a,b)]

def reference(y, u, mass, x, p, r, q, dt):
    """Original four-input model with corrected velocity; simple covariance update.

    The implementation uses RIM + Joseph form. This reference uses Decimal,
    a direct analytic inverse and (I-KC)*P_minus, independently of those paths.
    """
    with localcontext() as context:
        context.prec=70
        cast=lambda value:D.from_float(float(value))
        y,u,x=map(lambda a:list(map(cast,a)),(y,u,x))
        p,r,q=map(lambda a:[list(map(cast,row)) for row in a],(p,r,q))
        dt,mass=cast(dt),cast(mass)
        z,o=D(0),D(1)
        h=dt/mass
        f=[[o-h*x[2],-h*x[3],-h*x[0],-h*x[1]],
           [dt,o,z,z],[z,z,o,z],[z,z,z,o]]
        b=[[h,z,z,z],[z,dt,z,z],[z,z,dt,z],[z,z,z,dt]]
        predicted=[x[0]-h*(x[2]*x[0]+x[3]*x[1])+h*u[0],
                   x[1]+dt*x[0]+dt*u[1],x[2]+dt*u[2],x[3]+dt*u[3]]
        pm=plus(multiply(multiply(f,p),transpose(f)),multiply(multiply(b,q),transpose(b)))
        s=[[pm[i][j]+r[i][j] for j in range(2)] for i in range(2)]
        determinant=s[0][0]*s[1][1]-s[0][1]*s[1][0]
        inv=[[s[1][1]/determinant,-s[0][1]/determinant],
             [-s[1][0]/determinant,s[0][0]/determinant]]
        k=multiply([row[:2] for row in pm],inv)
        answer=[predicted[i]+sum(k[i][j]*(y[j]-predicted[j]) for j in range(2)) for i in range(4)]
        correction=[[D(i==j)-(k[i][j] if j<2 else z) for j in range(4)] for i in range(4)]
        result_p=multiply(correction,pm)
        return list(map(float,answer)),[list(map(float,row)) for row in result_p]

class TestEKF(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory=tempfile.TemporaryDirectory()
        cls.libraries=[]
        for optimization in ['-O0','-O3']:
            out=Path(cls.directory.name)/(optimization[1:]+'.so')
            subprocess.run([os.environ.get('CC','gcc'),'-std=c11','-pedantic',
                            '-Wall','-Wextra','-Werror',optimization,'-shared','-fPIC',
                            str(ROOT/'param_3link.c'),str(ROOT/'function_3link.c'),
                            str(ROOT/'RIM_functions.c'),'-lm','-o',str(out)],check=True)
            lib=c.CDLL(str(out))
            lib.EKF.argtypes=[c.POINTER(c.c_double),c.POINTER(c.c_double),c.c_double,
                             c.POINTER(c.c_double),c.POINTER(V4),c.POINTER(V2),
                             c.POINTER(V4),c.POINTER(c.c_double),c.POINTER(V4)]
            lib.EKF.restype=c.c_int
            cls.libraries.append(lib)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def invoke(self,lib,y,u,mass,x,p,r,q,alias=False):
        ay,au,ax=V2(*y),V4(*u),V4(*x)
        ap,ar,aq=matrix(M4,p),matrix(M2,r),matrix(M4,q)
        ox,op=(ax,ap) if alias else (V4(*([123.]*4)),matrix(M4,[[456.]*4]*4))
        result=lib.EKF(ay,au,mass,ax,ap,ar,aq,ox,op)
        return result,list(ox),[list(row) for row in op]

    def assert_close(self,actual,expected):
        for a,b in zip(actual,expected):
            self.assertTrue(math.isclose(a,b,rel_tol=3e-10,abs_tol=3e-12),(a,b))

    def test_parameter_values_and_types(self):
        expected=json.loads((ROOT/'tests/parameter_initial_bytes.json').read_text())['unchanged']
        for lib in self.libraries:
            for name,value in expected.items():
                self.assertEqual(bytes((c.c_char*(len(value)//2)).in_dll(lib,name)).hex(),value,name)
            for name,size in [('y_k',2),('u_k_1',4),('x_hat_k_1',4),('P_k_1',16),
                              ('Rkf',4),('Qkf',16),('x_hat_k',4),('P_k',16),('M_wy_k_1',1)]:
                self.assertEqual(list((c.c_double*size).in_dll(lib,name)),[0.]*size,name)

    def test_nonlinear_prediction_and_all_four_inputs(self):
        zero=[[0.]*4 for _ in range(4)]
        for lib in self.libraries:
            dt=c.c_float.in_dll(lib,'sampling_time').value
            result,x,p=self.invoke(lib,[0.,0.],[4.,5.,6.,7.],2.,[2.,3.,0.,0.],zero,[[1.,0.],[0.,1.]],zero)
            self.assertEqual(result,0)
            self.assert_close(x,[2.+2.*dt,3.+7.*dt,6.*dt,7.*dt])
            self.assertEqual(p,zero)

    def test_decimal_reference_and_inplace(self):
        randomizer=random.Random(73621)
        def spd(n,scale):
            a=[[randomizer.uniform(-1.,1.) for _ in range(n)] for _ in range(n)]
            return [[scale*sum(a[i][k]*a[j][k] for k in range(n))+(scale*.1 if i==j else 0.) for j in range(n)] for i in range(n)]
        for _ in range(120):
            y=[randomizer.uniform(-.5,.5),randomizer.uniform(-.2,.2)]
            u=[randomizer.uniform(-2.,2.) for _ in range(4)]
            mass=randomizer.uniform(.2,5.)
            x=[randomizer.uniform(-.5,.5),randomizer.uniform(-.2,.2),1.,100.]
            p,r,q=spd(4,.2),spd(2,.001),spd(4,.01)
            for lib in self.libraries:
                dt=c.c_float.in_dll(lib,'sampling_time').value
                ex,ep=reference(y,u,mass,x,p,r,q,dt)
                result,ax,ap=self.invoke(lib,y,u,mass,x,p,r,q)
                self.assertEqual(result,0)
                self.assert_close(ax,ex)
                for ar,er in zip(ap,ep):self.assert_close(ar,er)
                self.assertEqual(ap,transpose(ap))
                inplace=self.invoke(lib,y,u,mass,x,p,r,q,alias=True)
                self.assertEqual(inplace,(result,ax,ap))

    def test_repeated_estimation(self):
        for lib in self.libraries:
            dt=c.c_float.in_dll(lib,'sampling_time').value
            x=[0.,0.,1.,100.]
            p=[[1. if i==j else 0. for j in range(4)] for i in range(4)]
            q=[[.01 if i==j else 0. for j in range(4)] for i in range(4)]
            r=[[.001,0.],[0.,.0001]]
            for k in range(250):
                y=[.1*math.cos(k*.01),.01*math.sin(k*.01)]
                u=[math.sin(k*.01),0.,0.,0.]
                ex,ep=reference(y,u,2.,x,p,r,q,dt)
                result,x,p=self.invoke(lib,y,u,2.,x,p,r,q)
                self.assertEqual(result,0)
                self.assert_close(x,ex)
                for ar,er in zip(p,ep):self.assert_close(ar,er)
                self.assertTrue(all(p[i][i]>=0. for i in range(4)))

    def test_failures_do_not_modify_output(self):
        ident=[[1. if i==j else 0. for j in range(4)] for i in range(4)]
        zero=[[0.]*4 for _ in range(4)]
        for lib in self.libraries:
            defaults=dict(y=[0.,0.],u=[0.]*4,mass=1.,x=[0.,0.,1.,100.],p=ident,r=[[1.,0.],[0.,1.]],q=zero)
            cases=[({'mass':0.},-2),({'mass':-1.},-2),({'mass':math.nan},-2),
                   ({'y':[math.nan,0.]},-1),({'u':[0.,0.,math.inf,0.]},-1),
                   ({'p':zero,'r':[[0.,0.],[0.,0.]]},-4),
                   ({'r':[[-1.,0.],[0.,1.]]},-3),
                   ({'r':[[1.,2.],[2.,1.]]},-3),
                   ({'q':[[0.,1.,0.,0.],[1.,0.,0.,0.],[0.]*4,[0.]*4]},-3),
                   ({'r':[[1.,.1],[.2,1.]]},-3),
                   ({'p':zero,'x':[1e308,1e308,1e308,1e308]},-5)]
            for change,expected in cases:
                result,x,p=self.invoke(lib,**dict(defaults,**change))
                self.assertEqual(result,expected,change)
                self.assertEqual(x,[123.]*4)
                self.assertEqual(p,[[456.]*4]*4)
            for index in [0,1,3,4,5,6,7,8]:
                args=[V2(),V4(),1.,V4(),M4(),M2(),M4(),V4(),M4()]
                args[index]=None
                self.assertEqual(lib.EKF(*args),-1)

if __name__=='__main__':
    unittest.main(verbosity=2)

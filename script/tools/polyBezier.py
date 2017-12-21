from spline import bezier, bezier6, polynom, exact_cubic, curve_constraints, spline_deriv_constraint, from_bezier
import inspect

class PolyBezier:
    
    def __init__(self,curves):
        if not isinstance(curves,list): # deal with single bezier curve input
            curves = [curves]
        self.curves=curves
        self.times=[]
        self.times +=[0]
        for i in range(len(curves)):
            if not isinstance(curves[i],bezier):
                raise TypeError("PolyBezier must be called with a list of bezier curves (or a single bezier curve)")
            self.times+=[curves[i].max() + self.times[-1]]
            
            
    def __call__(self,t):
        if t>self.times[-1] or t<0:
            raise ValueError("Parameter is outside of definition range of the curves")
        for cit in range(len(self.times)):
            if t<=self.times[cit+1]:
                tc = t-self.times[cit]
                return self.curves[cit](tc)
            
    def length(self):
        return self.times[-1]
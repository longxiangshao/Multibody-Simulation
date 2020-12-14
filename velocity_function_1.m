function [v,s] = velocity_function_1(t,t0,t1,t2,t3,v0,dflag)
    if 0<=t && t<=t0
        v=0;
        s=0;
    end
    if t0<t && t<=t0+t1
        v=v0/t1*(t-t0);
        s=1/2*v0/t1*(t-t0)^2;
    end
    if t0+t1<t && t<=t0+t1+t2
        v=v0;
        s=1/2*v0*t1+v0*(t-t0-t1);
    end
    if t0+t1+t2<t && t<=t0+t1+t2+t3
        v=v0-v0/t3*(t-t0-t1-t2);
        s=1/2*v0*t1+v0*t2+v0*(t-t0-t1-t2)-1/2*v0/t3*(t-t0-t1-t2)^2;
    end
    if t>t0+t1+t2+t3
        v=0;
        s=1/2*v0*t1+v0*t2+1/2*v0*t3;
    end

    if dflag==1
        v=0;
        s=0;
    end
end



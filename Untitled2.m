t0 = 3;t1 = 2;t2 = 8;t3 = 2;
for i= 0:0.1:15
    if 0<=t && t<=t0
        v=0;
        %a=0;
        s=0;
    end
    if t0<t && t<=t0+t1
        v=v0*t^2*(3*t1-2*t)/t1^3;
        s=v0*t^3*(t1-1/2*t)/t1^3;
        %a=6*v0*t*(t1-t)/t1^3;
    end
    if t0+t1<t && t<=t0+t1+t2
        v=v0;
        s=1/2*v0*t1+v0*(t-t1);
        %a=0; 
    end
    if t0+t1+t2<t && t<=t0+t1+t2+t3
        v=v0*(t1+t2+t3-t)^2*(t3+2*t-2*t1-2*t2)/t3^3;
        s=1/2*v0*t1+v0*t2+v0*((t-t1-t2)-(t3*(t-t1-t2)^3-1/2*(t-t1-t2)^4)/t3^3);
        %a=v0*6*(t1+t2-t)*(t1+t2+t3-t)/t3^3;
    end
    if t>t0+t1+t2+t3
        v=0;
        s=1/2*v0*t1+v0*t2+v0*(t3-(t3*t3^3-1/2*t3^4)/t3^3);
        %a=0;
    end
    plot(t,v);
    plot(t,s);
end


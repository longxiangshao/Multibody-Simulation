function skew_a = skew(a)
a1 = a(1);
a2 = a(2);
a3 = a(3);

skew_a = [	  0,	-a3,	a2;
			 a3,	  0,   -a1;
			-a2,	 a1,     0];

end
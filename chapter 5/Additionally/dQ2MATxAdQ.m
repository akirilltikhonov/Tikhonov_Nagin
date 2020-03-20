 function dCdQ = dQ2MATxAdQ(Q,A)   
    
    dCdq1 = 2*[Q(1) -Q(4) Q(3); Q(4) Q(1) -Q(2); -Q(3) Q(2) Q(1)]*A;
    dCdq2 = 2*[Q(2) Q(3) Q(4); Q(3) -Q(2) -Q(1); Q(4) Q(1) -Q(2)]*A;
    dCdq3 = 2*[-Q(3) Q(2) Q(1); Q(2) Q(3) Q(4); -Q(1) Q(4) -Q(3)]*A;
    dCdq4 = 2*[-Q(4) -Q(1) Q(2); Q(1) -Q(4) Q(3); Q(2) Q(3) Q(4)]*A;
    dCdQ = [dCdq1 dCdq2 dCdq3 dCdq4];
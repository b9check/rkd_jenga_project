function frames = forward_kin(DH)
    n = length(DH(:,1)); %Robot DOF
    
    % Allocate a variable containing the transforms from each frame
    % to the base frame.
    frames = sym(zeros(4,4,n));            
    H = 1;

    for row = 1:n
        a = DH(row,1);
        alpha = DH(row,2);
        d = DH(row,3);
        theta = DH(row,4);

        H1_i = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)];
        H2_i = [sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)];
        H3_i = [0 sin(alpha) cos(alpha) d];
        H4_i = [0 0 0 1];
        H_i =  [H1_i; H2_i; H3_i; H4_i];

        H = H*H_i;
        frames(:,:,row) = H;

    end

end
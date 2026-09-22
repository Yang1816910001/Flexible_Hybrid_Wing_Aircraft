function c = cross3n(a, b) %#codegen
%CROSS3N  按列叉乘，a、b 都是 3×n
%
%  c(:,j) = a(:,j) × b(:,j)
%  节点力臂：M_i = cross3n(r_cg2node_b, F_b)，再 sum(M_i,2)+sum(M_node,2)

    c = [a(2, :).*b(3, :) - a(3, :).*b(2, :)
         a(3, :).*b(1, :) - a(1, :).*b(3, :)
         a(1, :).*b(2, :) - a(2, :).*b(1, :)];
end

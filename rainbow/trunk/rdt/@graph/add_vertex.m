function [T,ind] = add_node(T,x)
v = T.v;
T.v_ptr = T.v_ptr + 1;
v(T.v_ptr,:) = x';
T.v = v;
ind = T.v_ptr;
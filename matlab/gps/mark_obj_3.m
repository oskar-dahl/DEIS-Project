function [ outim ] = mark_obj(inim,ident_oo,nsc,pv,do_f)
%(C) Josef Bigun 2009, 2010
%MARK_OBJ Marks objects o, by tiny crosses in inim
%   ident_oo is a list of object coordinates in its rows. The
%   first column is row coordinate and the second column is column coordinate.
%   third column is spiral angles, fourth column is decimal identity
%   starting from 0.
%
%   Produces crosses, AND legible labels of identities at found spirals.


if nargin<3
    nsc=4;
end


if nargin<4
    pv=0;
end

if nargin<5
    do_f=2;
end
outim=inim;
o=ident_oo(:,1:3);
for k=1:size(o,1)
%     display([num2str(k) ': ' num2str([o(k,1),o(k,2)] )]);
o(k,1:3)=(round([ident_oo(k,1:2)/do_f ident_oo(k,3)]));
outim(o(k,1), (o(k,2)-nsc):(o(k,2)+nsc),2)=pv;
outim((o(k,1)-nsc):(o(k,1)+nsc),o(k,2),2)=pv;

position=[o(k,2),o(k,1)+6*nsc]; value=ident_oo(k,4);
outim = insertText(outim,position,value,'AnchorPoint','LeftBottom');
end

end


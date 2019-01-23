function [ outim ] = mark_obj(inim,oo,nsc,pv,do_f)
%(C) Josef Bigun 2009, 2010
%MARK_OBJ Marks objects o, by tiny crosses in inim
%   o is a list of object coordinates in its rows. The
%   first column is row coordinate and the second column is column coordinate.
%
%produces only crosses, no legible labels of identities.

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
o=oo;
for k=1:size(oo,1)
%     display([num2str(k) ': ' num2str([o(k,1),o(k,2)] )]);
o(k,1:3)=(round([oo(k,1:2)/do_f oo(k,3)]));
outim(o(k,1), (o(k,2)-nsc):(o(k,2)+nsc))=pv;
outim((o(k,1)-nsc):(o(k,1)+nsc),o(k,2))=pv;
end

end


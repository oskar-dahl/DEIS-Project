function [centroids] = centr(inim,thresh )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


C=bwconncomp(thresh<inim);
%[rw,cl]=ind2sub(C.ImageSize, C.PixelIdxList{1});
centroids=zeros(C.NumObjects,2);
for k=1:C.NumObjects
    pixi=C.PixelIdxList{k};
    inimv=inim(pixi);
    [row,col]=ind2sub(C.ImageSize, pixi);
    
    centroids(k,:)=  (inimv'/sum(inimv))*[row col];
end

end


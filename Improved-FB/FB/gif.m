function [Gif] = gif(frames)
dt=0.3;
     n=length(frames);
   for i=1:n
       [image,map]=frame2im(frames(i));
         [im,map2]=rgb2ind(image,128);
    if i==1
         imwrite(im,map2,'1.gif','gif','writeMode','overwrite','delaytime',dt,'loopcount',inf);
    else
        imwrite(im,map2,'1.gif','gif','writeMode','append','delaytime',dt); 
    end
end


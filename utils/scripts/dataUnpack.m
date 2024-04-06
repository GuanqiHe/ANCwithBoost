load("rejTemp.mat")
y = filter(hdbandpass, data.y);
plot((1:length(y))/25000,y*4);
legend(['y'])
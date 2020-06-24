clear all;
clc

[filename pathname]= uigetfile('*.dlf','Enter TIRF Test File','C:\')

SDdata = importdata([pathname filename]);

j=1;
for i=20:10:size(SDdata,1)
    partData(j,:) = SDdata(i,:);
    j = j+1;
end

for n=1:size(partData,1)
    remain = partData(n,:);
    for i=1:10
       [rawData(n,i), remain] = strtok(remain,'A');
    end 
end

% rawData = str2double(rawData);

newData = str2double(rawData);

% j=1;
% for i=1:10:size(rawData,1)
%     newData(j,:) = rawData(i,:);
%     j = j+1;
% end

for i=1:size(newData,1)
    rawGpsLat = -newData(i,2);
    degrees = fix(rawGpsLat/100);
    fraction = rawGpsLat - fix(rawGpsLat);
    minutes = 100*(((rawGpsLat - fraction)/100) - fix((rawGpsLat - fraction)/100)) + fraction;
    dm = [degrees minutes];
    newData(i,2) = -dm2degrees(dm);
    
    rawGpsLong = -newData(i,3);
    degrees = fix(rawGpsLong/100);
    fraction = rawGpsLong - fix(rawGpsLong);
    minutes = 100*(((rawGpsLong - fraction)/100) - fix((rawGpsLong - fraction)/100)) + fraction;
    dm = [degrees minutes];
    newData(i,3) = -dm2degrees(dm); 
    
    newData(i,4) = 1.852*newData(i,4);
    
    newData(i,5) = (newData(i,5)/2048);
    newData(i,6) = (newData(i,6)/2048);
    newData(i,7) = (newData(i,7)/2048);
end


% cat = strcat(col(1),'A',col(2),'A',col(3),'A',col(4),'A',col(5),'A',col(6),'A',col(7),'A',col(8),'A',col(9),'A',col(10))
% 
% 
% outData = [SDdata(1:4,:);newData]
fileID = fopen(strcat(strtok(filename,'.'),'_20hz.dlf'), 'wt')

dlmwrite(strcat(strtok(filename,'.'),'_20hz.dlf'), newData,'delimiter','A','precision',6)

str = strcat(char(35),'V2');
fprintf(fileID,'%s',str)
str = char(10);
fprintf(fileID,'%s',str)
str = strcat(char(35),"DATASTART")
fprintf(fileID,'%s',str)
str = char(10);
fprintf(fileID,'%s',str)
str = "Datalog Time; GPS Latitude; GPS Longitude; GPS Speed;Ax;Ay;Az;Gx;Gy;Gz;";
fprintf(fileID,'%s',str)
str = char(10);
fprintf(fileID,'%s',str)
str = "seg.; deg; deg; km/h;Gs.;Gs.;Gs.;un.;un.;un.;";
fprintf(fileID,'%s',str)




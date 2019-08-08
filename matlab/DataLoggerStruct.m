function DataLoggerStruct(s,name,video_flag,par,var,rec,ut)

for i=1:length(s)
  ss = s(i);
  filename = strcat('Results\',name,num2str(i),'.mat');
  save(filename,'-struct','ss');
  MakeVideo(ss,i,name,video_flag,par,var,rec,ut);
end

end
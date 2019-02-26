function length = CalcCableLen(sw_r,ang_t,pos_BA_glob)

  length = sw_r*(pi-ang_t)+sqrt(dot(pos_BA_glob,pos_BA_glob));
  
end
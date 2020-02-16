function [sig] = convert_modelsignal(class,name,time,value,status,rms)
sig.class = class;
sig.name = name;
sig.tov = time;
sig.value = value;
sig.status = status;
sig.rms = rms;
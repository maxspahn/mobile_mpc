function mpcProblem = readCppDumpFile(fileName)

a = load(fileName);

offset1 = 1;
offset2 = offset1 + 20;
offset3 = offset2 + (15 * 20);

mpcProblem.xinit = a(offset1:offset2-1);
mpcProblem.x0 = a(offset2:offset3-1);
mpcProblem.all_parameters = a(offset3:end);
end


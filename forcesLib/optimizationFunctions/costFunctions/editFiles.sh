for i in {2..15}
do
  file="costFunctionSimple_${i}.m"
  rm -f $file
  cp costFunctionSimple_1.m $file
  echo $file
  newFun="costFunctionSimple_${i}"
  sed -i "s/costFunctionSimple_1/costFunctionSimple_$i/" $file
  sed -i "s/i\s\=\s1/i \= $i/" $file
done

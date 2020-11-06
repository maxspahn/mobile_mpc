for i in {2..15}
do
  file="obstacleAvoidanceSphere_${i}.m"
  rm -f $file
  cp obstacleAvoidanceSphere_1.m $file
  echo $file
  newFun="obstacleAvoidanceSphere_${i}"
  sed -i "s/obstacleAvoidanceSphere_1/obstacleAvoidanceSphere_$i/" $file
  sed -i "s/t\s\=\s1/t \= $i/" $file
done

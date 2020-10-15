for i in {2..15}
do
  file="obstacleAvoidanceSimple_${i}.m"
  rm -f $file
  cp obstacleAvoidanceSimple_1.m $file
  echo $file
  newFun="obstacleAvoidanceSimple_${i}"
  sed -i "s/obstacleAvoidanceSimple_1/obstacleAvoidanceSimple_$i/" $file
  sed -i "s/t\s\=\s1/t \= $i/" $file
done

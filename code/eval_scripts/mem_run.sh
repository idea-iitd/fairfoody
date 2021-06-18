export IFS=$'\n'
for i in `cat "$1"`
do
	MEM=$(free -htw | grep "Mem:" | tr -s ' ' | cut -d ' ' -f 8 | tr -d 'G'| cut -d . -f 1)
	PROCESSES=$(ps aux | grep "main.o" | wc -l)
	echo $MEM
	echo $PROCESSES
	while [ $MEM -le 40 ] || [ $PROCESSES -ge $2 ]
	do
		MEM=$(free -htw | grep "Mem:" | tr -s ' ' | cut -d ' ' -f 8 | tr -d 'G'| cut -d . -f 1)
		PROCESSES=$(ps aux | grep "main.o" | wc -l)
   		echo "waiting for processes to decrease from $2"
   		sleep 10
	done
    echo $i > temp2.sh
    echo "taskset -cp 0-96 \$!" >> temp2.sh
    bash temp2.sh
    sleep 0.1
    # echo $i
done

# ./main.o AHM_results/0105/config_60 -ct EDT -algo HUNGARIAN_CLUSTER_PACKING_RESHUFFLING -mmc 10 -mid 1000000 -st 20 -end 22 >  AHM_results/0105/clustering_20-22_mid_10.results 

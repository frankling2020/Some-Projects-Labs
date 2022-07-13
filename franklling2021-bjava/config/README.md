# Hadoop
```bash
bin/hdfs namenode -format
sbin/start-all.sh
```

# Spark
---
```bash
spark-submit --class org.apache.spark.examples.SparkPi  --master yarn --deploy-mode cluster --driver-memory 1g  --executor-memory 1g --executor-cores 1 examples/jars/spark-examples*.jar 5
```

```bash
TERM=xterm-color spark-shell
```

# Zookeeper
```bash
bin/zkServer.sh start
```

# Drill
```bash
bin/drillbit.sh start
bin/sqlline -u jdbc:drill:zk=localhost,master:5181
```
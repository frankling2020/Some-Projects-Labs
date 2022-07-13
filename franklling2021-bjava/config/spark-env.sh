export HADOOP_HOME=/root/hadoop
export HADOOP_CONF_DIR=$HADOOP_HOME/etc/hadoop
export SPARK_DIST_CLASSPATH=$($HADOOP_HOME/bin/hadoop classpath)
export JAVA_HOME=$JAVA_HOME
export SPARK_MASTER_IP=$MASTER_IP
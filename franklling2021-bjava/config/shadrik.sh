# Hadoop Setup
export HDFS_NAMENODE_USER=root
export HDFS_DATANODE_USER=root
export HDFS_SECONDARYNAMENODE_USER=root
export YARN_RESOURCEMANAGER_USER=root
export YARN_NODEMANAGER_USER=root

export JAVA_HOME=/usr/lib/jvm/java-1.8.0-openjdk-amd64
export PDSH_RCMD_TYPE=ssh

export PATH=$PATH:$HADOOP_HOME/bin
export PATH=$PATH:$HADOOP_HOME/sbin
export HADOOP_HOME=/root/hadoop
export HADOOP_MAPRED_HOME=${HADOOP_HOME}
export HADOOP_COMMON_HOME=${HADOOP_HOME}
export HADOOP_HDFS_HOME=${HADOOP_HOME}
export YARN_HOME=${HADOOP_HOME}

export HADOOP_CONF_DIR=$HADOOP_HOME/etc/hadoop 
export SPARK_HOME=/root/spark
export PATH=$PATH:$SPARK_HOME/bin 
export LD_LIBRARY_PATH=${HADOOP_HOME}/lib/native

export MASTER_IP=$(hostname -I)
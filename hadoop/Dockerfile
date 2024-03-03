# Hadoop Base
FROM apache/hadoop:3

# Maintainer
LABEL frankling2021="carofrank2000@gmail.com"

# Spark Installation
RUN set -ex; \
    mkdir -p /opt/spark; \
    mkdir /opt/spark/python; \
    mkdir -p /opt/spark/examples; \
    mkdir -p /opt/spark/work-dir; \
    chmod g+w /opt/spark/work-dir;

# Install Apache Spark
# https://downloads.apache.org/spark/KEYS
ENV SPARK_TGZ_URL=https://archive.apache.org/dist/spark/spark-3.5.1/spark-3.5.1-bin-without-hadoop.tgz

RUN set -ex; \
    export SPARK_TMP="$(mktemp -d)"; \
    cd $SPARK_TMP; \
    wget -nv -O spark.tgz "$SPARK_TGZ_URL" --no-check-certificate; \
    tar -xf spark.tgz --strip-components=1; \
    mv jars /opt/spark/; \
    mv bin /opt/spark/; \
    mv sbin /opt/spark/; \
    mv kubernetes/dockerfiles/spark/decom.sh /opt/; \
    mv examples /opt/spark/; \
    mv kubernetes/tests /opt/spark/; \
    mv data /opt/spark/; \
    mv python/pyspark /opt/spark/python/pyspark/; \
    mv python/lib /opt/spark/python/lib/; \
    mv R /opt/spark/; \
    chmod a+x /opt/decom.sh; \
    cd ..; \
    rm -rf "$SPARK_TMP";

COPY entrypoint.sh /opt/

ENV SPARK_HOME /opt/spark
ENV HADOOP_HOME /opt/hadoop

WORKDIR /opt/spark/work-dir

USER root

ENTRYPOINT [ "/opt/entrypoint.sh" ]

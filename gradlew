#!/bin/sh

# Gradle start up script for POSIX systems

set -e

# Resolve APP_HOME
APP_HOME=$(cd "$(dirname "$0")" && pwd)

# Determine the Java command to use to start the JVM.
if [ -n "$JAVA_HOME" ] ; then
    if [ -x "$JAVA_HOME/jre/sh/java" ] ; then
        JAVACMD="$JAVA_HOME/jre/sh/java"
    else
        JAVACMD="$JAVA_HOME/bin/java"
    fi
    if [ ! -x "$JAVACMD" ] ; then
        die "ERROR: JAVA_HOME is set to an invalid directory: $JAVA_HOME"
    fi
else
    JAVACMD="java"
fi

WRAPPER_JAR="$APP_HOME/gradle/wrapper/gradle-wrapper.jar"
if [ ! -r "$WRAPPER_JAR" ]; then
    echo "Gradle wrapper jar not found, executing with local gradle if available..."
    exec gradle "$@"
fi

exec "$JAVACMD" -jar "$WRAPPER_JAR" "$@"

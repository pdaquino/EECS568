CXX = g++ -Wall -g
KINECT_OBJS = Kinect.o

LIB=libkinect.so
DESTLIB=../../lib/$(LIB)

all: $(DESTLIB)

$(DESTLIB): $(LIB)
	cp $(LIB) $(DESTLIB)

$(LIB): kinect_Kinect.h $(KINECT_OBJS)

kinect_Kinect.h:
	echo "Rebuilding JNI headers. Ensure java file has been recently built."
	javah -classpath ../kinect.jar kinect.Kinect

clean:
	rm -f $(KINECT_OBJS) *~ kinect_Kinect.h *.so $(LIB) $(DESTLIB)

JNI_INCLUDES = -I/usr/lib/jvm/java-6-usn/include/ -I/usr/lib/jvm/java-6-sun/include/linux -I/usr/lib/jvm/java-6-openjdk/include/
KINECT_INCLUDES = -I$(HOME)/libfreenect/include
KINECT_LIBS = -L$(HOME)/libfreenect/build/lib -lfreenect -lpthread

%.o: %.cpp
	$(CXX) -shared -O2 -fPIC $< $(JNI_INCLUDES) $(KINECT_INCLUDES) $(KINECT_LIBS) -o $(LIB)



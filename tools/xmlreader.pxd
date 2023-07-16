from libcpp.string cimport string

cdef extern from "xmlreader.h" namespace "${APP_NAME}":
    cdef cppclass XmlReader:
        const int STATUS_NONE
        const int STATUS_INIT
        XmlReader() except +
        XmlReader(const char* xmlFilePathName) except +
        XmlReader(const char* xmlFilePathName, char delimiter) except +
        
        int getStatus()
        
        char getDelimiter()
        void setDelimiter(char delimiter)

        const char* getXmlFilePathName()
        void setXmlFilePathName(const char* xmlFilePathName)

        int loadFile() except +
        int loadFile(const char* xmlFilePathName) except +
        int saveFile() except +
        int saveFile(const char* xmlFilePathName) except +

        const char* readXml() except +
        int writeXml(const char* xmlString) except +

        const char* getData(const char* path) except +
        const char* getData(const char* path, char delimiter) except +
        void putData(const char* path, const char* value) except +
        void putData(const char* path, const char* value, char delimiter) except +
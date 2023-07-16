from cython.operator cimport dereference as deref, postincrement as postinc
from libcpp.string cimport string
from xmlreader cimport XmlReader

cdef class PyXmlReader:
    cdef XmlReader *c_xml
    def __cinit__(self, xmlFilePathName=None, delimiter=None):
        if xmlFilePathName == None and delimiter == None:
            self.c_xml = new XmlReader()
        elif xmlFilePathName != None and delimiter == None:
            self.c_xml = new XmlReader(xmlFilePathName.encode('utf-8'))
        elif xmlFilePathName != None and delimiter != None:
            self.c_xml = new XmlReader(xmlFilePathName.encode('utf-8'), ord(delimiter))
        else:
            raise RuntimeError(f'Invalid Arguments: {xmlFilePathName}, {delimiter}')
    def __deallog__(self):
        del self.c_xml

    def getStatus(self):
        return self.c_xml.getStatus()

    def getDelimiter(self):
        return self.c_xml.getDelimiter().decode('utf-8')
    def setDelimiter(self, delimiter):
        self.c_xml.setDelimiter(ord(delimiter))

    def getXmlFilePathName(self):
        return self.c_xml.getXmlFilePathName().decode('utf-8')
    def setXmlFilePathName(self, xmlFilePathName):
        self.c_xml.setXmlFilePathName(xmlFilePathName.encode('utf-8'))

    def loadFile(self, xmlFilePathName=None):
        if xmlFilePathName == None:
            self.c_xml.loadFile()
        else:
            self.c_xml.loadFile(xmlFilePathName.encode('utf-8'))
    def saveFile(self, xmlFilePathName=None):
        if xmlFilePathName == None:
            self.c_xml.saveFile()
        else:
            self.c_xml.saveFile(xmlFilePathName.encode('utf-8'))

    def readXml(self):
        return self.c_xml.readXml().decode('utf-8')
    def writeXml(self, xmlString):
        return self.c_xml.writeXml(xmlString.encode('utf-8'))
    
    def getData(self, path, delimiter=None):
        if delimiter == None:
            return self.c_xml.getData(path.encode('utf-8')).decode('utf-8')
        else:
            return self.c_xml.getData(path.encode('utf-8'), ord(delimiter)).decode('utf-8')
    def putData(self, path, value, delimiter=None):
        if delimiter == None:
            self.c_xml.putData(path.encode('utf-8'), value.encode('utf-8'))
        else:
            self.c_xml.putData(path.encode('utf-8'), value.encode('utf-8'), ord(delimiter))
    
    @property
    def statusNone(self):
        return self.c_xml.STATUS_NONE
    @property
    def statusInit(self):
        return self.c_xml.STATUS_INIT
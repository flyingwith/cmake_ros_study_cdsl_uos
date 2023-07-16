#ifndef XMLREADER_H
#define XMLREADER_H

#include <cstdarg>
#include "tinyxml2.h"
#include "app.h"

namespace APP_NAMESPACE
{

// -----------------------------------------------------------------------------

class XmlReader
{
public:
    static const int STATUS_NONE = 0;
    static const int STATUS_INIT = 1;

    XmlReader()
        : XmlReader('.') {}
    XmlReader(char delimiter);
    XmlReader(const char* xmlFilePathName)
        : XmlReader(xmlFilePathName, '.') {}
    XmlReader(const char* xmlFilePathName, char delimiter);
    ~XmlReader();

    int getStatus(){ return m_status; }

    char getDelimiter(){ return m_delimiter; }
    void setDelimiter(char delimiter){ m_delimiter = delimiter; }
    
    const char* getXmlFilePathName(){ return m_xmlFilePathName.c_str(); }
    void setXmlFilePathName(const char* xmlFilePathName){ m_xmlFilePathName = xmlFilePathName; }

    int loadFile(){ return loadFile(m_xmlFilePathName.c_str()); }
    int loadFile(const char* xmlFilePathName);
    int saveFile(){ return saveFile(m_xmlFilePathName.c_str()); }
    int saveFile(const char* xmlFilePathName);

    const char* readXml();
    int writeXml(const char* xmlString);

    const char* getData(const char* path){ return getData(path, m_delimiter); }
    const char* getData(const char* path, char delimiter);
    void putData(const char *path, const char *value){ putData(path, value, m_delimiter); }
    void putData(const char *path, const char *value, char delimiter);

private:
    int m_status;
    std::string m_xmlFilePathName;
    char m_delimiter;
    tinyxml2::XMLDocument m_document;
    tinyxml2::XMLPrinter m_printer;
};

// -----------------------------------------------------------------------------

inline XmlReader::XmlReader(char delimiter)
    : m_status(STATUS_NONE)
    , m_xmlFilePathName("")
    , m_delimiter(delimiter)
{
    m_status = STATUS_INIT;
}
inline XmlReader::XmlReader(const char* xmlFilePathName, char delimiter)
    : m_status(STATUS_NONE)
    , m_xmlFilePathName(xmlFilePathName)
    , m_delimiter(delimiter)
{
    if (loadFile()) return;
    m_status = STATUS_INIT;
}
inline XmlReader::~XmlReader()
{
}

// -----------------------------------------------------------------------------

inline int XmlReader::loadFile(const char* xmlFilePathName)
{
    if (m_document.LoadFile(xmlFilePathName))
        throw std::runtime_error("Fail to load the xml file: " + std::string(xmlFilePathName));
    return 0;
}

inline int XmlReader::saveFile(const char* xmlFilePathName)
{
    if (m_document.SaveFile(xmlFilePathName))
        throw std::runtime_error("Fail to save the xml file: " + std::string(xmlFilePathName));
    return 0;
}

// -----------------------------------------------------------------------------

inline const char* XmlReader::readXml()
{
    m_document.Print(&m_printer);
    return m_printer.CStr();
}
inline int XmlReader::writeXml(const char* xmlString)
{
    if (m_document.Parse(xmlString) != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("Fail to write the xml string");
    return 0;
}

// -----------------------------------------------------------------------------

inline const char *XmlReader::getData(const char *path, char delimiter)
{
    std::istringstream iss(path);
    std::string node;
    tinyxml2::XMLElement *e = 0;
    const char* ret;
    bool attr = false;

    while (std::getline(iss, node, delimiter)) {
        if (!node.empty()) {
            if(attr == false) {
                if(node == "<xmlattr>") attr = true;
                else {
                    if (e == 0) e = m_document.FirstChildElement(node.c_str());
                    else e = e->FirstChildElement(node.c_str());
                    if (e == 0) throw std::runtime_error("Failed to find the node: " + node);
                }
            }
            else {
                ret = e->Attribute(node.c_str());
                if (ret == 0) throw std::runtime_error("Failed to get the value: " + std::string(path));
                return ret;
            }
        }
        else throw std::runtime_error("Failed to parse the path: " + std::string(path));
    }
    
    ret = e->GetText();
    if (ret == 0) throw std::runtime_error("Failed to get the value: " + std::string(path));
    return ret;
}

inline void XmlReader::putData(const char *path, const char *value, char delimiter)
{
    std::istringstream iss(path);
    std::string node;
    tinyxml2::XMLElement *e = 0;
    bool attr = false;

    while (std::getline(iss, node, delimiter)) {
        if (!node.empty()) {
            if(attr == false) {
                if(node == "<xmlattr>") attr = true;
                else {
                    if (e == 0) {
                        e = m_document.FirstChildElement(node.c_str());
                        if (e == 0) {
                            e = m_document.NewElement(node.c_str());
                            m_document.InsertFirstChild(e);
                        }
                    }
                    else {
                        tinyxml2::XMLElement *e_p = e;
                        e = e->FirstChildElement(node.c_str());
                        if (e == 0) {
                            e = m_document.NewElement(node.c_str());
                            e_p->InsertEndChild(e);
                        }
                    }
                }
            }
            else {
                e->SetAttribute(node.c_str(), value);
                return;
            }
        }
        else throw std::runtime_error("Failed to parse the path: " + std::string(path));
    }
    e->SetText(value);
}

// -----------------------------------------------------------------------------

} // namespace APP_NAMESPACE

#endif // XMLREADER_H
# -*- coding: utf-8 -*-

import pathlib, sys, os

# install
# |- tools
# |  |- py_xmlreader.so
# |  |- test_xmlreader
# |     |- test_xmlreader
# |     |- test_py_xmlreader

sys.path.append(str(pathlib.Path(os.getcwd()).parent))
import py_xmlreader 

def main():
    # =========================================================================
    print('\n'+'='*80+'\nread xml from file\n'+'='*80+'\n')
    
    print('\n<-- valid file name -->\n')

    try:
        xml = py_xmlreader.PyXmlReader('parameter.xml')
        print('Succeeded to read the xml file: ', xml.getXmlFilePathName())
        print(xml.readXml())
    except RuntimeError as err:
        print('Failed to read the xml file: ', err)
        return 1

    print('\n<-- invalid file name -->\n')

    try:
        xml = py_xmlreader.PyXmlReader('parameter0.xml')
        print('Succeeded to read the xml file: ', xml.getXmlFilePathName())
        print(xml.readXml())
    except RuntimeError as err:
        print('Failed to read the xml file: ', err)

    # =========================================================================
    print('\n'+'='*80+'\nread xml from string\n'+'='*80+'\n')
    
    print('\n<-- valid xml string -->\n')

    try:
        xml = py_xmlreader.PyXmlReader('parameter.xml')
        xml2 = py_xmlreader.PyXmlReader()
        xml2.writeXml(xml.readXml())
        print(xml2.readXml())
    except RuntimeError as err:
        print('Failed to write the xml string: ', err)
        return 1
    
    print('\n<-- invalid xml string -->\n')

    try:
        xml = py_xmlreader.PyXmlReader()
        xml.writeXml('hello world')
        print(xml.readXml())
    except RuntimeError as err:
        print('Failed to write the xml string: ', err)

    # =========================================================================
    print('\n'+'='*80+'\ntest getData and putData\n'+'='*80+'\n')

    try:
        xml = py_xmlreader.PyXmlReader('parameter.xml', '.')

        print('<-- get data -->')

        system_name = xml.getData('param.system.info.name')
        system_name_attr = xml.getData('param,system,info,<xmlattr>,name', ',')
        dim_x = int(xml.getData('param;system;dim;<xmlattr>;x', ';'))
        Kp = float(xml.getData('param control PID gains <xmlattr> Kp', ' '))
        print(f'system_name = {system_name}')
        print(f'system_name_attr = {system_name_attr}')
        print(f'dim_x = {dim_x}')
        print(f'Kp = {Kp}')

        print('<-- put data -->')

        xml.putData('param.system.info.name', 'my_system')
        xml.putData('param,system,info,<xmlattr>,name', 'my_system (attr)', ',')
        xml.putData('param;system;dim;<xmlattr>;x', '12', ';')
        xml.putData('param control PID gains <xmlattr> Kp', '110.5', ' ')
        
        system_name = xml.getData('param.system.info.name')
        system_name_attr = xml.getData('param.system.info.<xmlattr>.name')
        dim_x = int(xml.getData('param.system.dim.<xmlattr>.x'))
        Kp = float(xml.getData('param.control.PID.gains.<xmlattr>.Kp'))
        print(f'system_name = {system_name}')
        print(f'system_name_attr = {system_name_attr}')
        print(f'dim_x = {dim_x}')
        print(f'Kp = {Kp}')
    except RuntimeError as err:
        print('Failed to get and put data: ', err)
        return 1

    print('\n<-- wrong path for getData() -->\n')

    try:
        xml = py_xmlreader.PyXmlReader('parameter.xml')
        system_name = xml.getData('param.system.info.name2')
        print(f'system_name = {system_name}')
    except RuntimeError as err:
        print('Failed to get data: ', err)

    print('\n<-- new path for putData() -->\n')

    try:
        xml = py_xmlreader.PyXmlReader('parameter.xml')
        xml.putData('param.system.info.name2', 'my_system')
        xml.putData('param.system.info.<xmlattr>.name2', 'my_system (attr)')
        print(xml.readXml())
    except RuntimeError as err:
        print('Failed to put data: ', err)

    # =========================================================================
    print('\n'+'='*80+'\ncopy xml file\n'+'='*80+'\n')

    try:
        xml = py_xmlreader.PyXmlReader('parameter.xml')
        xml.putData('param.system.info.name', 'my_system')
        xml.putData('param.system.info.<xmlattr>.name', 'my_system (attr)')
        xml.saveFile('parameter2.xml')
        xml2 = py_xmlreader.PyXmlReader('parameter2.xml')
        print(xml2.readXml())
        os.remove('parameter2.xml')
    except RuntimeError as err:
        print('Failed to copy the xml file: ', err)
        return 1
    
    # =========================================================================
    print('\n'+'='*80+'\ncreate xml file\n'+'='*80+'\n')

    try:
        xml = py_xmlreader.PyXmlReader()
        xml.putData('param.test.<xmlattr>.name', 'Hello World!')
        xml.putData('param.test.name', 'Goodbye World!')
        xml.saveFile('parameter2.xml')
        xml2 = py_xmlreader.PyXmlReader('parameter2.xml')
        xml2.loadFile('parameter2.xml')
        print(xml2.readXml())
        os.remove('parameter2.xml')
    except RuntimeError as err:
        print('Failed to create the xml file: ', err)
        return 1

    return 0

if __name__ == '__main__':
    main()
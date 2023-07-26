#!/usr/bin/env python3

import sys
import re
import string

sys.path.append("/usr/lib/freecad-python3/lib")
sys.path.append("/usr/lib/freecad/Mod")
sys.path.append("/usr/lib/freecad/bin")
sys.path.append("/usr/lib/freecad/Ext")
sys.path.append("/usr/lib/freecad/Gui")

import pyvista as pv
import FreeCAD as App
import FreeCADGui as Gui
import Part
import Mesh
import pyvista as pv
from PySide import QtCore, QtGui
import pandas as pd
import os

import subprocess
class MainWindow(QtGui.QMainWindow):
    def showEvent(self, event):
        Gui.setupWithoutGUI()

class ReverseEngineering():
    def __init__(self):
        #Prepare ReverseEngineering-SimpleGUI
        if not QtGui.QApplication.instance():
            self.app=QtGui.QApplication(sys.argv)
        else:
            self.app = QtGui.QApplication.instance()
        self.mw=MainWindow()
        self.mw.resize(1,1)
        self.mw.show()
        self.part = ''
        self.diameter = ''
        self.names_list = []
        # list all .ipt files
        current_path = os.path.dirname(os.path.abspath(__file__))+'/Banhrang/ipts/ipts'
        file_type='.ipt'
        
        self.ipts_list = self.access_files(current_path,file_type)

        # list all .stp files
        current_path = os.path.dirname(os.path.abspath(__file__))+'/Banhrang/ipts/stps'
        file_type='.stp'
        
        self.stp_list = self.access_files(current_path,file_type)
        # print(self.stp_list)

        self.part_available = False



    def access_files(self,directory,file_type):
        files = []
        idx = 1
        
        for item in os.listdir(directory):
            item_path = os.path.join(directory, item)
            if os.path.isfile(item_path) and item.endswith(file_type):
                if file_type == '.ipt':
                    # print(str(idx)+'.',item)
                    self.names_list.append(f'{str(idx)}.{item}')
                    # print(self.names_list[idx-1])
                    idx+=1
                files.append(item.replace(file_type, ''))
        
        return files

    def parameters(self,part_name):

        current_path = os.path.dirname(os.path.abspath(__file__))+'/Banhrang/ipts/ipts/'+part_name+'.ipt'
        # print('mm'+current_path)
        import importerIL
        importerIL.open(current_path)
        # print('\n\n'+current_path+'\n\n')
            
        punctuation_chars = re.escape(string.punctuation)
        doc_name = re.sub(r'[\s' + punctuation_chars + ']', '_', part_name)

        doc = App.getDocument(str(doc_name)).getObject('Parameters')
        # Create a list to store the parameter data
        data = []
            
        for param in doc.PropertiesList:
            
            param_parameter = ''
            param_value = ''
            param_formula = ''
            param_tolerance = ''
            param_comment = ''
            
            if doc.getPropertyByName(param) == 'Parameter':
                continue

            elif param.startswith('A'):
                if any(param[1] == str(x) for x in range(1, 10)):
                    param_parameter = str(doc.getPropertyByName(param))
                    
                    try:
                        B_index = 'B'+param[1]+param[2]
                        C_index = 'C'+param[1]+param[2]
                        param_value = str(doc.getPropertyByName(B_index))
                        param_formula = str(doc.getPropertyByName(C_index))
                        
                        try:
                            D_index = 'D'+param[1]+param[2]
                            param_tolerance = str(doc.getPropertyByName(D_index))
                        
                        except AttributeError:
                            param_tolerance = ''

                        
                        try:
                            E_index = 'E'+param[1]+param[2]
                            param_comment = str(doc.getPropertyByName(E_index))
                        
                        except AttributeError:
                            param_comment = ''


                            

                    except IndexError:
                        B_index = 'B'+param[1]
                        C_index = 'C'+param[1]
                        param_value = str(doc.getPropertyByName(B_index))
                        param_formula = str(doc.getPropertyByName(C_index))
                        
                        try:
                            D_index = 'D'+param[1]
                            param_tolerance = str(doc.getPropertyByName(D_index))
                        
                        except AttributeError:
                            param_tolerance = ''

                        
                        try:
                            E_index = 'E'+param[1]
                            param_comment = str(doc.getPropertyByName(E_index))
                        
                        except AttributeError:
                            param_comment = ''
                    
                    data.append([param_parameter, param_value, param_formula, param_tolerance, param_comment])


                else:
                    pass
            else:
                pass
                

        # Create a pandas DataFrame from the parameter data
        self.df = pd.DataFrame(data, columns=['Parameter', 'Value', 'Formula', 'Tolerance', 'Comment'])
        if len(data) == 20:
            part = 'shaft'
            diameter = self.df.loc[self.df['Parameter'] == 'd3', 'Value'].values[0]
        
        elif len(data) == 38:
            part = 'gear'
            diameter = self.df.loc[self.df['Parameter'] == 'd49', 'Value'].values[0]
            

        elif len(data) == 41:
            part = 'nut'
            diameter = self.df.loc[self.df['Parameter'] == 'THREADDIA', 'Value'].values[0]

        elif len(data) == 41:
            part = 'bearing'
            diameter = self.df.loc[self.df['Parameter'] == 'THREADDIA', 'Value'].values[0]

        elif len(data) == 44:
            part = 'bolt'
            diameter = self.df.loc[self.df['Parameter'] == 'THREADDIA', 'Value'].values[0]
        # self.df = self.df.to_string()

        # Print the DataFrame
        print(self.df)       
        
        
        # print("Part: ",part,"\nDiameter: "+str(diameter))
        # print("DATA",len(data))



        self.df.to_excel('output.xlsx', index=False)
        print("Part parameters extracted and saved successfully.")

        self.part = part
        self.diameter = diameter

    def show_part(self,part_name):
        shape = Part.Shape()
        part_name = os.path.dirname(os.path.abspath(__file__))+'/Banhrang/ipts/stps/'+part_name+'.stp'
        shape.read(part_name)
        doc = App.newDocument('Doc')
        pf = doc.addObject("Part::Feature","MyShape")
        pf.Shape = shape
        Mesh.export([pf], 'part.stl')
        shape = pv.read('part.stl')
        plotter = pv.Plotter()
        plotter.add_mesh_clip_box(shape, color='white')
        plotter.add_silhouette(shape,color='black')
        plotter.add_axes()
        plotter.enable_trackball_style()
        # plotter.enable_eye_dome_lighting()
        plotter.show()


    def show_draw_and_assembly(self):
        location = os.path.dirname(os.path.abspath(__file__))+'/Banhrang/ipts/ipts/'
        files = os.listdir(location)
        pdf_files = [file for file in files if file.endswith('.pdf')]
        full_name=''
        for file in pdf_files:
            full_name = os.path.join(location, file)
            # print(full_name)
        subprocess.Popen(['evince',full_name])
        shape = pv.read(location+'Assembly5.stl')

        plotter = pv.Plotter()
        plotter.add_mesh_clip_box(shape, color='white')
        plotter.add_silhouette(shape,color='black')
        plotter.add_axes()
        plotter.enable_trackball_style()
        # plotter.enable_eye_dome_lighting()
        plotter.show()

    def full_parts_list(self):
        idx = 1
        self.parts_list = []
        for part_name in self.ipts_list:

            current_path = os.path.dirname(os.path.abspath(__file__))+'/Banhrang/ipts/ipts/'+part_name+'.ipt'
            import importerIL
            importerIL.open(current_path)
            # print('\n\n'+current_path+'\n\n')
                
            punctuation_chars = re.escape(string.punctuation)
            doc_name = re.sub(r'[\s' + punctuation_chars + ']', '_', part_name)

            doc = App.getDocument(str(doc_name)).getObject('Parameters')
            # Create a list to store the parameter data
            data = []
                
            for param in doc.PropertiesList:
                
                param_parameter = ''
                param_value = ''
                param_formula = ''
                param_tolerance = ''
                param_comment = ''
                
                if doc.getPropertyByName(param) == 'Parameter':
                    continue

                elif param.startswith('A'):
                    if any(param[1] == str(x) for x in range(1, 10)):
                        param_parameter = str(doc.getPropertyByName(param))
                        
                        try:
                            B_index = 'B'+param[1]+param[2]
                            C_index = 'C'+param[1]+param[2]
                            param_value = str(doc.getPropertyByName(B_index))
                            param_formula = str(doc.getPropertyByName(C_index))
                            
                            try:
                                D_index = 'D'+param[1]+param[2]
                                param_tolerance = str(doc.getPropertyByName(D_index))
                            
                            except AttributeError:
                                param_tolerance = ''

                            
                            try:
                                E_index = 'E'+param[1]+param[2]
                                param_comment = str(doc.getPropertyByName(E_index))
                            
                            except AttributeError:
                                param_comment = ''


                                

                        except IndexError:
                            B_index = 'B'+param[1]
                            C_index = 'C'+param[1]
                            param_value = str(doc.getPropertyByName(B_index))
                            param_formula = str(doc.getPropertyByName(C_index))
                            
                            try:
                                D_index = 'D'+param[1]
                                param_tolerance = str(doc.getPropertyByName(D_index))
                            
                            except AttributeError:
                                param_tolerance = ''

                            
                            try:
                                E_index = 'E'+param[1]
                                param_comment = str(doc.getPropertyByName(E_index))
                            
                            except AttributeError:
                                param_comment = ''
                        
                        data.append([param_parameter, param_value, param_formula, param_tolerance, param_comment])


                    else:
                        pass
                else:
                    pass
                    

            # Create a pandas DataFrame from the parameter data
            self.df = pd.DataFrame(data, columns=['Parameter', 'Value', 'Formula', 'Tolerance', 'Comment'])
            if len(data) == 20:
                part = 'shaft'
                diameter = self.df.loc[self.df['Parameter'] == 'd3', 'Value'].values[0]
            
            elif len(data) == 38:
                part = 'gear'
                diameter = self.df.loc[self.df['Parameter'] == 'd49', 'Value'].values[0]
                

            elif len(data) == 41:
                part = 'nut'
                diameter = self.df.loc[self.df['Parameter'] == 'THREADDIA', 'Value'].values[0]

            elif len(data) == 41:
                part = 'bearing'
                diameter = self.df.loc[self.df['Parameter'] == 'THREADDIA', 'Value'].values[0]

            elif len(data) == 44:
                part = 'bolt'
                diameter = self.df.loc[self.df['Parameter'] == 'THREADDIA', 'Value'].values[0]

            part_info = [str(idx),part_name,part,diameter]
            self.parts_list.append(part_info)
            idx+=1




if __name__ == '__main__':

    rev = ReverseEngineering()
    # rev.full_parts_list()
    # rev.show_draw_and_assembly()
    # print("PARTS:",rev.parts_list)


    idx = int(input("Enter which part you want: "))
    if 0<idx<=len(rev.ipts_list):
        rev.parameters(rev.ipts_list[idx-1])
        rev.show_part(rev.ipts_list[idx-1])


    else:
        print('Wrong selection.....\n')



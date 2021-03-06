#!/home/turtlebot/anaconda3/envs/gui/bin/python3
from ctypes import sizeof
from numpy import size
import wx
from strings import *
import rospy
import time

"""
* Filename: gui_gameboi.py
* Student: Harsh Deshpande, hdeshpande@ucsd.edu; Daniel Beaglehole, dbeaglehole@ucsd.edu; Divyam Bapna, dbapna@ucsd.edu; Chao Chi Cheng, cccheng@ucsd.edu
* Project #6:  GameBoi
*
* Description: This file contains code for the GUI used for Gameboi. It uses the wxpython library to make an 
                interactive GUI capable of receving text input as well as communicating with the main controller.
                The gui has a background image, a robot avatar image (captrain america ftw), a tex box where all
                text outputs are formatted and displayed and finally an instructions panel which stays on screen
                to provide instructions for special keys like enter to continue or escape to exit.
                It also contains code for displaying the survey forms and retrieving the entered values from the
                participants. These are then rallied to the main controller.
*
*How to use:
* Build:
*   catkin build
*   source ~/catkin_ws/devel/setup.bash
* Usage:
*   rosrun gameboi gui_gameboi.py
* Requirement:
*   wxpython should be installed
*   Make sure every python files permission is set properly
"""

class GameFlags: #Use this for all game related flags
    def __init__(self):
        self.enter_pressed = False
        self.curr_game = 0

#------------------GUI Code--------------------
class MyPanel(wx.Panel):
    #----------------------------------------------------------------------
    def __init__(self, parent):
        """Constructor"""
        wx.Panel.__init__(self, parent)

        parent.ShowFullScreen(True)
        img_path = '/home/turtlebot/chocolate_ws/src/gameboi/image'
        #---store background data---
        bg_img = f'{img_path}/back.jpeg'
        self.bg = wx.Image(bg_img, wx.BITMAP_TYPE_ANY)
        self.SetBackgroundStyle(wx.BG_STYLE_ERASE)
        self.bgh = self.bg.GetHeight()
        self.bgw = self.bg.GetWidth()

        #-----Store Avatar data------
        avtar_img = f'{img_path}/camerica.jpeg'
        self.avt = wx.Image(avtar_img, wx.BITMAP_TYPE_ANY)
        self.avh = self.avt.GetHeight()
        self.avw = self.avt.GetWidth()

        #-----------Panel Display Flags------
        self.Instruction = False
        self.started = True
        self.evt = None
        self.new_test = False
        self.text = None
        self.inst = None
        self.size_changed = None
        self.bold_text = False

        #---------GameFlags------------------
        self.GAMEFLAGS = GameFlags()
        self.Bind(wx.EVT_KEY_DOWN, self.onKey)
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)
        self.Bind(wx.EVT_SIZE,self.OnSize)
        self.Bind(wx.EVT_IDLE,self.OnIdle)
        
    
    #----------------------------------------------------------------------
    def OnEraseBackground(self, evt):
        """
        Add a picture to the background
        """
        # yanked from ColourDB.py
        dc = evt.GetDC() if evt is not None else None
        self.evt = evt
                
        if not dc:
            dc = wx.ClientDC(self)
            rect = self.GetUpdateRegion().GetBox()
            # dc.SetClippingRect(rect)

        cliWidth, cliHeight = self.GetClientSize()
        # print(cliWidth,cliHeight)
        dc.Clear()

        # Calculate scale factors
        # The final image will be distorted. Change this maths for "fit in window", but leaving margins
        scaledimage = self.bg.Scale(cliWidth, cliHeight)
        dc.DrawBitmap(wx.Bitmap(scaledimage), 0, 0)

        #-------------------Display Avatar here if started---------------------
        if self.started:
            cliWidth, cliHeight = self.GetClientSize()
            scaledimage = self.avt.Scale(cliWidth//3, cliHeight*9//10)
            dc.DrawBitmap(wx.Bitmap(scaledimage),cliWidth//20,cliHeight//20)
            self.inst
        print(self.Instruction)
        if not self.Instruction:
            if self.inst: self.inst.Destroy()
            if not self.inst:
                cliWidth, cliHeight = self.GetClientSize()
                pos = (16*cliWidth//20 , 17*cliHeight//20)
                self.inst = wx.StaticText(self, label=inst_string, pos=pos)
                font1 = wx.Font(15, wx.MODERN, wx.NORMAL, wx.NORMAL, False)
                self.inst.SetForegroundColour((0,0,0))
                self.inst.SetFont(font1)
                self.Instruction = True
        
        if self.new_test:
            cliWidth, cliHeight = self.GetClientSize()
            pos = (10*cliWidth//20 , 4*cliHeight//20)
            if self.text: self.text.Destroy()
            self.text = wx.StaticText(self, label=self.text_str, pos=pos)
            font_size = 20
            font1 = wx.Font(font_size, wx.MODERN, wx.NORMAL, wx.NORMAL, False)

            if self.bold_text:
                font_size = 30
                font1 = wx.Font(font_size, wx.MODERN, wx.NORMAL, wx.NORMAL, False)
                
            font1 = font1.Bold()
            self.text.SetForegroundColour((0,0,0))
            self.text.SetBackgroundColour((225,225,225))
            self.text.SetFont(font1)
            self.new_test = False

        #-------------------Display Text Box Here------------------------------
        if self.started:
            assert 1==1
    
    def update_display(self,exc_inst=False): #Setting it to true excludes instructions
        self.Instruction = exc_inst
        self.OnEraseBackground(None)
    
    def update_text(self,prompt,make_bold = False):
        #Receives String
        # print("Updating Text")
        self.text_str = prompt
        self.new_test = True
        self.bold_text = make_bold
        self.update_display(exc_inst=True)

    #-------------Used for communicating flags with main------------------
    def get_enter_flag(self):
        if self.GAMEFLAGS.enter_pressed:
            self.GAMEFLAGS.enter_pressed = False
            return True
        else:
            return False
    
    def make_enter_false(self):
        self.GAMEFLAGS.enter_pressed = False
        return
        
    #----------------------------------------------------------------------
    def onKey(self, event):
        """
        Keyboard Input Interpretations
        """
        key_code = event.GetKeyCode()
        if key_code == wx.WXK_ESCAPE:
            self.GetParent().Close()
        elif key_code == wx.WXK_RETURN:
            # print("Enter Pressed")
            if not self.started:
                self.started = True
                # print("Start received")
                self.update_display()
            elif not self.GAMEFLAGS.enter_pressed:
                self.GAMEFLAGS.enter_pressed = True
        elif key_code == wx.WXK_BACK:
            # print("right received")
            # self.text_str = get_colour_order()
            # self.new_test = True
            # self.update_display(exc_inst=True)
            self.GetParent().show_form()
        else:
            event.Skip()

    def OnSize(self, event):
        self.size_changed = True

    def OnIdle(self,event):
        if self.size_changed: 
            print("New size:", self.GetSize())
            
            self.update_display(exc_inst=False)

            self.size_changed = None


#--------------Wapper Class for GUI and Survey Modules-----------
class Puzzle_Bot_GUI(wx.Frame):
    #-------------------------USAGE--------------------------------
    """
        Use self.panel.update_text(arg) with argument as the string you want to display
    """
    def __init__(self):
        """Constructor"""
        wx.Frame.__init__(self, None, title="Test FullScreen")
        self.form_panel = None
        self.panel = MyPanel(self)
        self.previous = None
        self.Show()

    def show_pre_form(self): #Call this to display the survey form
        # self.previous = self.panel
        # self.panel.Close() #Destroy the other panel
        self.form_panel = TestFrame(self)
        self.form_panel.Show()
        return

    def show_post_form(self): #Call this to display the survey form
        # self.previous = self.panel
        # self.panel.Close() #Destroy the other panel
        self.form_panel = TestFrame1(self)
        self.form_panel.Show()
        return
    
    def show_game(self): #Call this to restart the game
        self.form_panel.Destroy()
        self.form_panel = None
        # wx.Frame.__init__(self, None, title="Test FullScreen")
        # self.panel = MyPanel(self)
        # self.Bind(wx.EVT_KEY_DOWN, self.on_press)
        # self.ShowFullScreen(True)
        # self.panel.Show()
    
    def on_press(self, event):
        value = self.text_ctrl.GetValue()
        if not value:
            print("You didn't enter anything!")
        else:
            print(f'You typed: "{value}"')


#-------------Pre-Survey Frame Codes-----------
class TestFrame(wx.Frame):
    def __init__(self,parent):
        wx.Frame.__init__(self, parent, -1, "Please take a few seconds to rate your experience")
        panel = wx.Panel(self)
        self.button_clicked = False
        self.saved_rating = None
        self.Bind(wx.EVT_KEY_DOWN, self.onKey)

        # First create the controls
        topLbl = wx.StaticText(panel, -1, "Survey Questions")
        topLbl.SetFont(wx.Font(23, wx.SWISS, wx.NORMAL, wx.BOLD))

        # self.comm_sub = rospy.Publisher('/survey', String, queue_size=1)
        

        nameLbl = wx.StaticText(panel, -1, "Name: Optional")
        self.name = wx.TextCtrl(panel, -1, "")

        instLbl = wx.StaticText(panel, -1, "On a scale of 1-5 how much do you agree with the following:")
        instLbl.SetFont(wx.Font(10, wx.SWISS, wx.NORMAL, wx.BOLD))

        ratingLbl1 = wx.StaticText(panel, -1, "I feel comforted being with robots that have emotion")
        self.rating1 = wx.TextCtrl(panel, -1, "")

        ratingLbl2 = wx.StaticText(panel, -1, "I would feel very nervous just standing in front of a robot")
        self.rating2 = wx.TextCtrl(panel, -1, "")

        ratingLbl3 = wx.StaticText(panel, -1, "I would feel paranoid talking with a robot")
        self.rating3 = wx.TextCtrl(panel, -1, "")

        addrLbl = wx.StaticText(panel, -1, "Comments: Optional")
        self.addr1 = wx.TextCtrl(panel, -1, "")

        self.saveBtn = wx.Button(panel, -1, "Save")
        self.saveBtn.Bind(wx.EVT_BUTTON,self.OnSaveClick)
        self.cancelBtn = wx.Button(panel, -1, "Cancel")
        self.cancelBtn.Bind(wx.EVT_BUTTON,self.OnCancelClick) 

        mainSizer = wx.BoxSizer(wx.VERTICAL)
        mainSizer.Add(topLbl, 0, wx.ALL, 5)
        mainSizer.Add(wx.StaticLine(panel), 0,wx.EXPAND|wx.TOP|wx.BOTTOM, 5)


        addrSizer1 = wx.FlexGridSizer(cols=2, hgap=3, vgap=3)
        addrSizer1.AddGrowableCol(1)
        addrSizer1.Add(nameLbl, 0,
                wx.ALIGN_RIGHT|wx.ALIGN_CENTER_VERTICAL)
        addrSizer1.Add(self.name, 0, wx.EXPAND)
        mainSizer.Add(addrSizer1, 0, wx.EXPAND|wx.ALL, 10)

        mainSizer.Add(instLbl, 0, wx.ALL, 5)
        mainSizer.Add(wx.StaticLine(panel), 0,wx.EXPAND|wx.TOP|wx.BOTTOM, 5)

        addrSizer = wx.FlexGridSizer(cols=2, hgap=3, vgap=3)
        # addrSizer.AddGrowableCol(1)
        mainSizer.Add(ratingLbl1, 0, wx.ALL, 5)
        mainSizer.Add(self.rating1, 0, wx.ALL,5)

        mainSizer.Add(ratingLbl2, 0, wx.ALL, 5)
        mainSizer.Add(self.rating2, 0, wx.ALL,5)

        mainSizer.Add(ratingLbl3, 0, wx.ALL, 5)
        mainSizer.Add(self.rating3, 0, wx.ALL,5)

        addrSizer.Add(addrLbl, 0,
                wx.ALIGN_RIGHT|wx.ALIGN_CENTER_VERTICAL)
        addrSizer.Add(self.addr1, 0, wx.EXPAND)
        addrSizer.Add((10,10)) # some empty space

        mainSizer.Add(addrSizer, 0, wx.EXPAND|wx.ALL, 10)


        btnSizer = wx.BoxSizer(wx.HORIZONTAL)
        btnSizer.Add((20,20), 1)
        btnSizer.Add(self.saveBtn)
        btnSizer.Add((20,20), 1)
        btnSizer.Add(self.cancelBtn)
        btnSizer.Add((20,20), 1)

        mainSizer.Add(btnSizer, 0, wx.EXPAND|wx.BOTTOM, 10)
        panel.SetSizer(mainSizer)
    
    def OnSaveClick(self,event):
        # btn = event.GetEventObject().GetLabel() 
        # print("Label of pressed button = ",btn)
        value = self.name.GetValue() + "|" + self.rating1.GetValue() + "|" + self.rating2.GetValue() + "|" + self.rating3.GetValue() + "|" + self.addr1.GetValue() + "\n"
        self.saved_rating = [self.rating1.GetValue(),self.rating2.GetValue(),self.rating3.GetValue()]
        fileh = open('/home/turtlebot/chocolate_ws/src/gameboi/survey/pre_Survey_out.txt',"a")
        fileh.write(value)
        fileh.close()
        # self.comm_sub.publish("enter")
        self.button_clicked = True

        # self.GetParent().show_game()
    
    def OnCancelClick(self,event):
        # self.comm_sub.publish("enter")
        self.button_clicked = True
        # self.GetParent().show_game()
    
    def onKey(self, event):
        key_code = event.GetKeyCode()
        if key_code == wx.WXK_ESCAPE:
            self.GetParent().Close()

#------------Pos-Survey Frame Codes------------
class TestFrame1(wx.Frame):
    def __init__(self,parent):
        wx.Frame.__init__(self, parent, -1, "Please take a few seconds to rate your experience")
        panel = wx.Panel(self)
        self.button_clicked = False
        self.saved_rating = None
        self.Bind(wx.EVT_KEY_DOWN, self.onKey)

        # First create the controls
        topLbl = wx.StaticText(panel, -1, "Survey Questions")
        topLbl.SetFont(wx.Font(23, wx.SWISS, wx.NORMAL, wx.BOLD))

        # self.comm_sub = rospy.Publisher('/survey', String, queue_size=1)
        

        nameLbl = wx.StaticText(panel, -1, "Name: Optional")
        self.name = wx.TextCtrl(panel, -1, "")

        instLbl = wx.StaticText(panel, -1, "On a scale of 1-5 how much do you agree with the following:")
        instLbl.SetFont(wx.Font(10, wx.SWISS, wx.NORMAL, wx.BOLD))

        

        ratingLbl1 = wx.StaticText(panel, -1, "Rate Your Experience With Robot")
        self.rating1 = wx.TextCtrl(panel, -1, "")

        ratingLbl2 = wx.StaticText(panel, -1, "Do you think the game is interesting?")
        self.rating2 = wx.TextCtrl(panel, -1, "")

        ratingLbl3 = wx.StaticText(panel, -1, "How human do you think the robot is?")
        self.rating3 = wx.TextCtrl(panel, -1, "")

        self.saveBtn = wx.Button(panel, -1, "Save")
        self.saveBtn.Bind(wx.EVT_BUTTON,self.OnSaveClick)
        self.cancelBtn = wx.Button(panel, -1, "Cancel")
        self.cancelBtn.Bind(wx.EVT_BUTTON,self.OnCancelClick) 

        mainSizer = wx.BoxSizer(wx.VERTICAL)
        mainSizer.Add(topLbl, 0, wx.ALL, 5)
        mainSizer.Add(wx.StaticLine(panel), 0,wx.EXPAND|wx.TOP|wx.BOTTOM, 5)


        addrSizer1 = wx.FlexGridSizer(cols=2, hgap=3, vgap=3)
        addrSizer1.AddGrowableCol(1)
        addrSizer1.Add(nameLbl, 0,
                wx.ALIGN_RIGHT|wx.ALIGN_CENTER_VERTICAL)
        addrSizer1.Add(self.name, 0, wx.EXPAND)
        mainSizer.Add(addrSizer1, 0, wx.EXPAND|wx.ALL, 10)

        mainSizer.Add(instLbl, 0, wx.ALL, 5)
        mainSizer.Add(wx.StaticLine(panel), 0,wx.EXPAND|wx.TOP|wx.BOTTOM, 5)

        addrSizer = wx.FlexGridSizer(cols=2, hgap=3, vgap=3)
        # addrSizer.AddGrowableCol(1)
        mainSizer.Add(ratingLbl1, 0, wx.ALL, 5)
        mainSizer.Add(self.rating1, 0, wx.ALL,5)

        mainSizer.Add(ratingLbl2, 0, wx.ALL, 5)
        mainSizer.Add(self.rating2, 0, wx.ALL,5)

        mainSizer.Add(ratingLbl3, 0, wx.ALL, 5)
        mainSizer.Add(self.rating3, 0, wx.ALL,5)


        btnSizer = wx.BoxSizer(wx.HORIZONTAL)
        btnSizer.Add((20,20), 1)
        btnSizer.Add(self.saveBtn)
        btnSizer.Add((20,20), 1)
        btnSizer.Add(self.cancelBtn)
        btnSizer.Add((20,20), 1)

        mainSizer.Add(btnSizer, 0, wx.EXPAND|wx.BOTTOM, 10)
        panel.SetSizer(mainSizer)
    
    def OnSaveClick(self,event):
        # btn = event.GetEventObject().GetLabel() 
        # print("Label of pressed button = ",btn)
        value = self.name.GetValue() + "|" + self.rating1.GetValue() + "\n"
        self.saved_rating1 = (self.rating1.GetValue(),self.rating2.GetValue(),self.rating3.GetValue())
        self.saved_rating = self.rating1.GetValue()
        
        fileh = open('/home/turtlebot/chocolate_ws/src/gameboi/survey/post_Survey_out.txt',"a")
        fileh.write(value)
        fileh.close()
        # self.comm_sub.publish("enter")
        self.button_clicked = True

        # self.GetParent().show_game()
    
    def OnCancelClick(self,event):
        # self.comm_sub.publish("enter")
        self.button_clicked = True
        # self.GetParent().show_game()
    
    def onKey(self, event):
        key_code = event.GetKeyCode()
        if key_code == wx.WXK_ESCAPE:
            self.GetParent().Close()

#--------------Overall Wrapper Class that initialized a Puzzle_Bot_GUI object with utility functions like cleanup()
class gui:
    def __init__(self):
        """Constructor"""
        self.app = wx.App()
        self.frame = Puzzle_Bot_GUI()
    
    def start(self):
        self.app.MainLoop()

    def stop(self):
        def _cleanup():
            for tlw in wx.GetTopLevelWindows():
                if tlw:
                    tlw.Destroy()
            wx.WakeUpIdle()
        wx.CallLater(50, _cleanup)
        del self.app
        wx.Exit()

if __name__ == '__main__':
    rospy.init_node('GameBoiGui')
    app = wx.App()
    frame = Puzzle_Bot_GUI()
    app.MainLoop()
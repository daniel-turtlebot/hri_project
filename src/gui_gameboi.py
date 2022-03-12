#!/home/turtlebot/anaconda3/envs/gui/bin/python3
from ctypes import sizeof
from numpy import size
import wx
from strings import *
import rospy
import time

class GameFlags: #Use this for all game related flags
    def __init__(self):
        self.enter_pressed = False
        self.curr_game = 0

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
            font_size = 15
            font1 = wx.Font(font_size, wx.MODERN, wx.NORMAL, wx.NORMAL, False)
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
    
    def update_text(self,prompt):
        #Receives String
        # print("Updating Text")
        self.text_str = prompt
        self.new_test = True
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

    def show_form(self): #Call this to display the survey form
        # self.previous = self.panel
        # self.panel.Close() #Destroy the other panel
        self.form_panel = TestFrame(self)
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

class TestFrame(wx.Frame):
    def __init__(self,parent):
        wx.Frame.__init__(self, parent, -1, "Please take a few seconds to rate your experience")
        panel = wx.Panel(self)
        self.Bind(wx.EVT_KEY_DOWN, self.onKey)

        # First create the controls
        topLbl = wx.StaticText(panel, -1, "Survey Questions")
        topLbl.SetFont(wx.Font(18, wx.SWISS, wx.NORMAL, wx.BOLD))

        # self.comm_sub = rospy.Publisher('/survey', String, queue_size=1)
        self.button_clicked = False
        self.saved_rating = None

        nameLbl = wx.StaticText(panel, -1, "Name:")
        self.name = wx.TextCtrl(panel, -1, "")

        ratingLbl = wx.StaticText(panel, -1, "Rating:")
        self.rating = wx.TextCtrl(panel, -1, "")

        addrLbl = wx.StaticText(panel, -1, "Comments:")
        self.addr1 = wx.TextCtrl(panel, -1, "")

        self.saveBtn = wx.Button(panel, -1, "Save")
        self.saveBtn.Bind(wx.EVT_BUTTON,self.OnSaveClick)
        self.cancelBtn = wx.Button(panel, -1, "Cancel")
        self.cancelBtn.Bind(wx.EVT_BUTTON,self.OnCancelClick) 

        mainSizer = wx.BoxSizer(wx.VERTICAL)
        mainSizer.Add(topLbl, 0, wx.ALL, 5)
        mainSizer.Add(wx.StaticLine(panel), 0,wx.EXPAND|wx.TOP|wx.BOTTOM, 5)

        addrSizer = wx.FlexGridSizer(cols=2, hgap=5, vgap=5)
        addrSizer.AddGrowableCol(1)
        addrSizer.Add(nameLbl, 0,
                wx.ALIGN_RIGHT|wx.ALIGN_CENTER_VERTICAL)
        addrSizer.Add(self.name, 0, wx.EXPAND)
        addrSizer.Add(ratingLbl, 0,
                wx.ALIGN_RIGHT|wx.ALIGN_CENTER_VERTICAL)
        addrSizer.Add(self.rating, 0, wx.EXPAND)
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
        value = self.name.GetValue() + "|" + self.rating.GetValue() + "|" + self.addr1.GetValue() + "\n"
        self.saved_rating = self.rating.GetValue()
        fileh = open('Survey_out.txt',"a")
        fileh.write(value)
        fileh.close()
        # self.comm_sub.publish("enter")
        self.button_clicked = True

        # self.GetParent().show_game()
    
    def OnCancelClick(self,event):
        self.comm_sub.publish("enter")
        self.button_clicked = True
        # self.GetParent().show_game()
    
    def onKey(self, event):
        key_code = event.GetKeyCode()
        if key_code == wx.WXK_ESCAPE:
            self.GetParent().Close()

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
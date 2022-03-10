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
        self.Bind(wx.EVT_KEY_DOWN, self.onKey)
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)
        self.Bind(wx.EVT_SIZE,self.OnSize)
        self.Bind(wx.EVT_IDLE,self.OnIdle)

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
            if not self.started:
                self.started = True
                # print("Start received")
                self.update_display()
            elif not self.GAMEFLAGS.enter_pressed:
                self.GAMEFLAGS.enter_pressed = True
        elif key_code == wx.WXK_BACK:
            # print("right received")
            self.text_str = get_colour_order()
            self.new_test = True
            self.update_display(exc_inst=True)
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
        self.panel = MyPanel(self)
        self.Show()
    
    def on_press(self, event):
        value = self.text_ctrl.GetValue()
        if not value:
            print("You didn't enter anything!")
        else:
            print(f'You typed: "{value}"')

class gui:
    def __init__(self):
        """Constructor"""
        self.app = wx.App()
        self.frame = Puzzle_Bot_GUI()
    
    def start(self):
        self.app.MainLoop()

if __name__ == '__main__':
    rospy.init_node('GameBoiGui')
    app = wx.App()
    frame = Puzzle_Bot_GUI()
    # rospy.spin()
    app.MainLoop()
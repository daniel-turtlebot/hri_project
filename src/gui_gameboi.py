from ctypes import sizeof
from numpy import size
import numpy as np
# from pandas import StringDtype
import wx
from strings import *
import sys
# import rospy

class MyPanel(wx.Panel):
    #----------------------------------------------------------------------
    def __init__(self, parent):
        """Constructor"""
        wx.Panel.__init__(self, parent)
        self.Bind(wx.EVT_KEY_DOWN, self.onKey)
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)

        #---store background data---
        bg_img = 'back.jpeg'
        self.bg = wx.Image(bg_img, wx.BITMAP_TYPE_ANY)
        self.SetBackgroundStyle(wx.BG_STYLE_ERASE)
        self.bgh = self.bg.GetHeight()
        self.bgw = self.bg.GetWidth()

        #-----Store Avatar data------
        avtar_img = 'camerica.jpeg'
        self.avt = wx.Image(avtar_img, wx.BITMAP_TYPE_ANY)
        self.avh = self.avt.GetHeight()
        self.avw = self.avt.GetWidth()

        #---------Rospy Subscriber-----------
        # self.command = rospy.Subscriber('/gui_commands', String, self.command_callback)

        #-----------Panel Display Flags------
        self.Instruction = False
        self.started = False
        self.evt = None
        self.new_test = False
    
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
        dc.Clear()

        # Calculate scale factors
        # The final image will be distorted. Change this maths for "fit in window", but leaving margins
        scaledimage = self.bg.Scale(cliWidth, cliHeight)
        dc.DrawBitmap(wx.Bitmap(scaledimage), 0, 0)
        print("reached")

        #-------------------Display Avatar here if started---------------------
        if self.started:
            scaledimage = self.avt.Scale(cliWidth//3, cliHeight*9//10)
            dc.DrawBitmap(wx.Bitmap(scaledimage),cliWidth//20,cliHeight//20)
            
        if not self.Instruction:
            pos = (10*cliWidth//20 , 16*cliHeight//20)
            self.inst = wx.StaticText(self, label=inst_string, pos=pos)
            font1 = wx.Font(40, wx.MODERN, wx.NORMAL, wx.NORMAL, False)
            self.inst.SetForegroundColour((0,0,0))
            self.inst.SetFont(font1)
            self.Instruction = True
        
        if self.new_test:
            print("Printing text box")
            pos = pos = (10*cliWidth//20 , 4*cliHeight//20)
            self.text = wx.StaticText(self, label=self.text_str, pos=pos)
            font_size = 30
            font1 = wx.Font(font_size, wx.MODERN, wx.NORMAL, wx.NORMAL, False)
            self.text.SetForegroundColour((0,0,0))
            self.text.SetBackgroundColour((225,225,225))
            self.text.SetFont(font1)
            self.new_test = False

        #-------------------Display Text Box Here------------------------------
        if self.started:
            assert 1==1
        
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
                print("Start received")
                self.OnEraseBackground(None)
        elif key_code == wx.WXK_BACK:
            print("right received")
            self.text_str = get_colour_order()
            self.new_test = True
            self.OnEraseBackground(None)
        else:
            event.Skip()
        
class MyFrame(wx.Frame):
    """"""
    #----------------------------------------------------------------------
    def __init__(self):
        """Constructor"""
        wx.Frame.__init__(self, None, title="Test FullScreen")
        panel = MyPanel(self)
        self.ShowFullScreen(True)
        self.Show()
    
    def on_press(self, event):
        value = self.text_ctrl.GetValue()
        if not value:
            print("You didn't enter anything!")

if __name__ == '__main__':
    app = wx.App()
    frame = MyFrame()
    app.MainLoop()
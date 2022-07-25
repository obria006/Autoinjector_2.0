"""Scripts for interfacing with ZEN to automate microscope movemetns"""
import win32com.client

class ZEN():
    """Interface for ZEN application"""
    def __init__(self):
        self.zen = win32com.client.GetActiveObject("Zeiss.Micro.Scripting.ZenWrapperLM")
        print(self.zen)

if __name__ == "__main__":
    z = ZEN()
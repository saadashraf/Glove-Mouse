   M S B A M L   `   `   `  ����D  @GyroMouse, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null%clr-namespace:GyroMouse	GyroMouse  5   	       GyroMouse.MainWindow   5   	   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   @d2http://schemas.microsoft.com/expression/blend/2008     5   	   Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     5   	   )localclr-namespace:GyroMouse     5   	     � Title$  	GyroMouse��5   	   $	��400��6   $	��600��6(    � 
ResizeMode$ CanMinimize=�64    � WindowStartupLocation$ CenterScreen=�6M   .��5	      � ��5
   
   �� 5       �� ImageSource$&  Resource/GyroMouseBackground.png��6    � RowDefinitions 5   
   �� 5      $
��169* �6   �� 5      $	��16* �6   .+�5   
   �� -   5         ��Name$ Button_Connect��5      $��CONNECT��6)   $��Right=�6;   $��0,0,443.6,241.4q�6W   $��Bottom=�6p   $	��109��6�   $��30��6�   $��Franklin Gothic Heavy)�6�   $
��Bold"�6�   $��18(�6�   ���	#FF00A896 � ToolTip$ Click to connect��6G  ��6  Z� ��60  Z� 6G  �� 5   
   -   5        $ Button_Disconnect��5      $��
DISCONNECT��6,   $
��Left=�6A   $��
25,217,0,0q�6\   $	��Top=�6p   $	��129��6�   $��29��6�   $��Franklin Gothic Heavy)�6�   $
��Bold"�6�   $��18(�6�    J� RenderTransformOrigin$ 2.021,1.271&�6  ���	#FF00A896$ Click to disconnect��6k  ��6>  Z� ��6T  Z� 6k  �� 5   
   -   5         ��Name$ CheckBox_YInverted��5      $��Invert Y-Axis��6/   $
��Left=�6G   $��
33,155,0,0q�6b   $	��Top=�6v   $��18��6�   $	��118��6�   $��Franklin Gothic Heavy)�6�   $
��Bold"�6  $��16(�6)  ���	#FF8ECFE5���	#FF00A896$ Invert vertical movement��6|  ��67  Z� 6|  �� 5   
   -   5        $ Button_Refresh��5      $��REFRESH��6)   $��Right=�6;   $��0,0,20.6,57.4q�6W   $��Bottom=�6n   $	��129��6�   $��29��6�   $��Franklin Gothic Heavy)�6�   $
��Bold"�6�   $��18(�6�   ���	#FF00A896$ Refresh port list��6E  ��6  Z� ��6.  Z� 6E  �� 5   
   -   5        	 ��Name$	 ListBox_Ports��5      $
��Left=�6)   $	��108��6D   $	��Top=�6Q   $	��118��6i   $��Franklin Gothic Heavy)�6u   $��18(�6�   $��442,103,0,0q�6�   ���	#FF00A896��6�   Z� ��6�   Z� �� 5   
   $��Ports��6   $��Right=�6    $��0,30,47.6,0q�6<   $	��Top=�6Q   $��36��6i   $��72��6u   $��Franklin Gothic Heavy)�6�   $��24(�6�   ���	#FF00A896���	#FFB6DCF6$ COM port list��6�   �� 5   
   $��5,6.6,4.6,5.4q�6   $��1��6.   $
��Auto=�6;   .��5      �� -   5        
 ��Name$
 TextBlock_Log��5      $
��Wrap=�6/   $�� ��6C   $��1,1,1,1q�6K   ��5      �� 5       �� EndPoint$ 0.5,1&�6*    �� 
StartPoint$ 0.5,0&�6;   .��5      �  � Color$ 	#FFCAFFF9��6'    � Offset$ 0Q�69   � 5      $ White��6'   $ 1Q�65   5      5      5       �� 5!   
   -   5        $ CheckBox_Safety��5!      $��Safety��6,   $
��Left=�6=   $��
33,178,0,0q�6X   $	��Top=�6l   $��18��6�   $��78��6�   $��Franklin Gothic Heavy)�6�   $��16(�6  ���	#FF8ECFE5���	#FF00A896$E ?Turn on and take mouse pointer to top left corner to disconnect��6Y  ��6  Z� 6Y  �� 5"   
   -   5         ��Name$ Rectangle_Status��5"      
���  ��$
��Left=�6=   $��47��6X   $��268,0,0,48.4q�6d   $��Bottom=�6z   $��51��6�   $ 14.292,5.45&�6�   $+ %Green: Connected | Gray: Disconnected��6�   ��5#      �� 5$      $ 0.5,1&�6&   $ 0.5,0&�67   .��5%      � $ 	#FF00544B��6#   $ 1Q�65   � 5&      $ 	#FFA0D4EF��6#   5'      5)      �� 5*   
   -	   5        $ Button_Help��5*      $��!��6&   $
��Left=�62   $��215,247,0,0q�6M   $	��Top=�6b   $��33��6z   $��27��6�   $��Franklin Gothic Heavy)�6�   $
��Bold"�6�   $��18(�6�   $ 2.021,1.271&�6�   ���	#FF00A896$
 Help��6V  ��6)  Z� ��6?  Z� 6V  �� 5+   
   -
   5        $ Button_About��5+      $��?��6'   $
��Left=�63   $��339,247,0,0q�6N   $	��Top=�6c   $��33��6{   $��27��6�   $��Franklin Gothic Heavy)�6�   $
��Bold"�6�   $��18(�6�   $ 2.021,1.271&�6�   ���	#FF00A896$ About��6X  ��6+  Z� ��6A  Z� 6X  �� 5,   
   -   5        $ CheckBox_ClickMode��5,      $��
Click Mode��6/   $
��Left=�6D   $��
33,132,0,0q�6_   $	��Top=�6s   $��18��6�   $	��105��6�   $��Franklin Gothic Heavy)�6�   $
��Bold"�6  $��16(�6&  ���	#FF8ECFE5���	#FF00A896$ Invert vertical movement��6y  ��64  Z� 6y  5.      5/      
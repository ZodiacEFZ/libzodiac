package frc.libzodiac.ui;
import java.util.Arrays;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

class RecArr {
    public RecArr(){}
    public String arr2str(XboxRecorder[] recList) {
        StringBuilder sb = new StringBuilder();
        for (XboxRecorder recorder : recList) {
            sb.append(recorder.a()).append(",");
            sb.append(recorder.b()).append(",");
            sb.append(recorder.x()).append(",");
            sb.append(recorder.y()).append(",");
            sb.append(recorder.leftBumper()).append(",");
            sb.append(recorder.rightBumper()).append(",");
            sb.append(recorder.start()).append(",");
            sb.append(recorder.back()).append(",");
            sb.append(recorder.getLeftTriggerAxis()).append(",");
            sb.append(recorder.getRightTriggerAxis()).append(",");
            sb.append(recorder.getLeftX()).append(",");
            sb.append(recorder.getLeftY()).append(",");
            sb.append(recorder.getRightX()).append(",");
            sb.append(recorder.getRightY()).append("\n");
        }
        return sb.toString();
    }
    public XboxRecorder[] str2arr(String serializedString) {
        String[] lines = serializedString.split("\n");
        XboxRecorder[] recList = new XboxRecorder[lines.length];
        
        for (int i = 0; i < lines.length; i++) {
            String[] fields = lines[i].split(",");
            boolean a = Boolean.parseBoolean(fields[0]);
            boolean b = Boolean.parseBoolean(fields[1]);
            boolean x = Boolean.parseBoolean(fields[2]);
            boolean y = Boolean.parseBoolean(fields[3]);
            boolean leftBumper = Boolean.parseBoolean(fields[4]);
            boolean rightBumper = Boolean.parseBoolean(fields[5]);
            boolean start = Boolean.parseBoolean(fields[6]);
            boolean back = Boolean.parseBoolean(fields[7]);
            double leftTriggerAxis = Double.parseDouble(fields[8]);
            double rightTriggerAxis = Double.parseDouble(fields[9]);
            double leftX = Double.parseDouble(fields[10]);
            double leftY = Double.parseDouble(fields[11]);
            double rightX = Double.parseDouble(fields[12]);
            double rightY = Double.parseDouble(fields[13]);
            
            recList[i] = new XboxRecorder(a, b, x, y, leftBumper, rightBumper, start, back, leftTriggerAxis, rightTriggerAxis, leftX, leftY, rightX, rightY);
            recList[i].Round();
        }
        
        return recList;
    }
 
}
class XboxRecorder {
    private transient final double DIV = 10000.0;
    
    public XboxRecorder(boolean a,boolean b,boolean x,boolean y,boolean leftBumper,boolean rightBumper,boolean start,boolean back,double leftTriggerAxis,double rightTriggerAxis,double leftX,double leftY,double rightX,double rightY) {
        this._a=a;
        this._b=b;
        this._x=x;
        this._y=y;
        this._leftBumper=leftBumper;
        this._rightBumper=rightBumper;
        this._start=start;
        this._back=back;
        this._LeftTriggerAxis=leftTriggerAxis;
        this._RightTriggerAxis=rightTriggerAxis;
        this._LeftX=leftX;
        this._LeftY=leftY;
        this._RightX=rightX;
        this._RightY=rightY;
    }
    
    private boolean _a;
    public boolean a(){return this._a;}
    private boolean _b;
    public boolean b(){return this._b;}
    private boolean _x;
    public boolean x(){return this._x;}
    private boolean _y;
    public boolean y(){return this._y;}
    private boolean _leftBumper;
    public boolean leftBumper(){return this._leftBumper;}
    private boolean _rightBumper;
    public boolean rightBumper(){return this._rightBumper;}
    private boolean _start;
    public boolean start(){return this._start;}
    private boolean _back;
    public boolean back(){return this._back;}
    private double _LeftTriggerAxis;
    public double getLeftTriggerAxis(){return this._LeftTriggerAxis;}
    private double _RightTriggerAxis;
    public double getRightTriggerAxis(){return this._RightTriggerAxis;}
    private double _LeftX;
    public double getLeftX(){return this._LeftX;}
    private double _LeftY;
    public double getLeftY(){return this._LeftY;}
    private double _RightX;
    public double getRightX(){return this._RightX;}
    private double _RightY;
    public double getRightY(){return this._RightY;}
    public XboxRecorder Round() {
        this._LeftTriggerAxis=(Math.round(this._LeftTriggerAxis * DIV) / DIV);
        this._RightTriggerAxis=(Math.round(this._RightTriggerAxis * DIV) / DIV);
        this._LeftX=(Math.round(this._LeftX * DIV) / DIV);
        this._LeftY=(Math.round(this._LeftY * DIV) / DIV);
        this._RightX=(Math.round(this._RightX * DIV) / DIV);
        this._RightY=(Math.round(this._RightY * DIV) / DIV);
        return this;
    }

}
/*
public class App {
    public static void main(String[] args) {
        XboxRecorder[] recList = new XboxRecorder[5];
        // 初始化recList，这里假设所有值都为初始值（false和0.0）
        Arrays.fill(recList, new XboxRecorder(false, false, false, false, false, false, false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        RecArr recArr=new RecArr();
        String serializedString = recArr.arr2str(recList);
        System.out.println(serializedString);
        XboxRecorder[] recList1 = recArr.str2arr(serializedString);
        
        // 打印数组以验证反序列化结果
        for (XboxRecorder recorder : recList1) {
            System.out.println(recorder);
        }        

        }
}
 */
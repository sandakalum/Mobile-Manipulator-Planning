package com.github.irvs.ros_tms.tms_ur.tms_ur_arviewer.data;

//センサの値をメインに伝達するためだけのクラス
//正直いらない

public class Com {

	public enum COMMAND{LEFT, CENTER, RIGHT, UP, DOWN};
	
	private COMMAND com;
	
	public Com(){
		com = COMMAND.CENTER;
	}
	
	public void setCommand(COMMAND command){
		com = command;
	}
	
	public COMMAND getCommand(){
		return com;
	}
	
	@Override
	public String toString(){
		
		switch(com){
		
		case CENTER:
			return "CENTER";
		case LEFT:
			return "LEFT";
		case RIGHT:
			return "RIGHT";
		case UP:
			return "UP";
		case DOWN:
			return "DOWN";
		default:
			return "DEFAULT";	
		}
	}
	
}

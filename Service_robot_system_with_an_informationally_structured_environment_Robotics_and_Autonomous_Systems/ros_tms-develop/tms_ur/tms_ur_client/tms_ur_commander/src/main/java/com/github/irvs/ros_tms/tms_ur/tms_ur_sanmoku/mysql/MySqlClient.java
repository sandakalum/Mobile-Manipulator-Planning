package com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.mysql;

import android.content.Context;
import android.os.AsyncTask;
import android.util.Log;
import android.widget.Toast;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.Statement;
import java.util.ArrayList;

public class MySqlClient extends AsyncTask<Void, Void, Void> {
	//mysql IP:192.168.4.112
	private final String IPADDR = "192.168.4.131";
	private final String CLASSNAME = "MySqlClient";
	private boolean result = false;
	
	private String query;
	private String name, pass;
	public ArrayList<String> strs;
	
	Context con = null;
	
	public MySqlClient(Context context, String username, String password){
		name = username;
		pass = password;
		con = context;
		Log.v(CLASSNAME,"usr:"+username+"pass:"+password);
	}
	
	//クエリを直接書く
	public void setQuery(String str){
		query = str;
	}
	//クエリ入力補助
	public void setTagResearchQuery(String word){
		query = "select tag from rostms_tag where segments like '%;"+word+";%'";
	}
	
	public boolean getResult(){
		return result;
	}
	
	@Override
	protected Void doInBackground(Void... params) {
		result = false;
		strs = null;
		strs = new ArrayList<String>();
		
		Log.v(CLASSNAME, "param:" + con.toString() + ":" + IPADDR + ":" + name + ":" + pass);
		
		if(query == null) return null;
		try{
			Class.forName("com.mysql.jdbc.Driver");
			Log.v(CLASSNAME,"classForName comp");
			Connection conn = DriverManager.getConnection("jdbc:mysql://" + IPADDR + "/rostms_android2?useUnicode=true&characterEncoding=SJIS", name, pass);
//			Connection conn = DriverManager.getConnection("jdbc:mysql://192.168.4.114/rostms_android", name, pass);
			Log.v(CLASSNAME,"Connection comp");
			Statement stmt = conn.createStatement();
			Log.v(CLASSNAME,query);
			ResultSet rs = stmt.executeQuery(query);
			while(rs.next()){
				strs.add(rs.getString(1));
			}
			rs.close();
			stmt.close();
			conn.close();
			result = true;
		} catch(Exception e){
			Log.v(CLASSNAME,"mysql error");
			result = false;
		}
		
		for(String str : strs){
			Log.v(CLASSNAME, str);
		}
		
		return null;
	}
	
	protected void onPostExecute(){
        if(result) {
        	Toast.makeText(con,"Successed!", Toast.LENGTH_LONG).show();
        	Log.v(CLASSNAME, "Success onPostExec");
        }
        else {
        	Toast.makeText(con,"Failed!", Toast.LENGTH_LONG).show();
        	Log.v(CLASSNAME, "Failed onPostExec");
        }
        
    }
	
}

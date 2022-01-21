package com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.mysql;

import android.util.Log;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;

public class GetTagFromMysql{
	
	private String IPADDR;
	private String USER;
	private String PASS;
	
	private final String DBNAME = "rostms_android2";
	
	public GetTagFromMysql(String ip, String user, String pass){
		
		IPADDR = ip;
		USER = user;
		PASS = pass;
		
	}
	
	public String getTag(String word){
		String result = "";
		
		try {
			Class.forName("com.mysql.jdbc.Driver");
			Connection conn = DriverManager.getConnection("jdbc:mysql://" + IPADDR + "/" + DBNAME + "?useUnicode=true&characterEncoding=SJIS",USER,PASS);
			Statement stmt = conn.createStatement();
			String query = "select tag from rostms_android2.rostms_tag where segments like '%;"+word+";%';";
			
			ResultSet rs = stmt.executeQuery(query);
			Log.v("msql","query:" + query);
			if(rs.next()){
				result += rs.getString(1);
				Log.v("msql",result);
			}
			else{
				result = null;
				Log.v("msql", "No return info.");
			}
			rs.close();
			stmt.close();
			conn.close();
		} catch (ClassNotFoundException e) {
			Log.v("msql","classNotFoundEx");
			e.printStackTrace();
		} catch (SQLException e) {
			Log.v("msql","sqlEx");
			e.printStackTrace();
		}
		
		return result;
	}

}

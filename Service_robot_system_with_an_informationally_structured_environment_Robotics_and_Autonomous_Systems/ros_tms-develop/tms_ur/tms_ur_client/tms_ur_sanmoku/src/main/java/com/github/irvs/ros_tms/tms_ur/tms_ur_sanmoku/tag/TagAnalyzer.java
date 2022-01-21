package com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.tag;

import android.content.Context;
import android.util.Log;
import android.widget.Toast;

import com.github.irvs.ros_tms.tms_ur.tms_ur_sanmoku.mysql.GetTagFromMysql;

import java.util.ArrayList;


public class TagAnalyzer {
	private final static String CLASSNAME = "TagAnalyzer";
	private static String IPADDR = "";
	private static String USER = "", PASS = "";
	
	public TagAnalyzer(String ip, String usr, String pass){
		IPADDR= ip;
		USER = usr;
		PASS = pass;
	}

	//文章を動詞、形容詞、名詞の読み列に変換
	//原型に変換
	public static ArrayList<String> divideSentence(String key){
		//dstの初期化
		ArrayList<String> dst = new ArrayList<String>();
		if(key.length() > 0) { 
			for(net.reduls.sanmoku.Morpheme e : net.reduls.sanmoku.Tagger.parse(key)) {
				net.reduls.sanmoku.FeatureEx fe = new net.reduls.sanmoku.FeatureEx(e);
				if(e.feature.indexOf("名詞")==0||e.feature.indexOf("形容詞")==0){
					dst.add(fe.reading);
				}
				else if(e.feature.indexOf("動詞")==0){
					net.reduls.sanmoku.Morpheme sub_e = net.reduls.sanmoku.Tagger.parse(fe.baseform).get(0);
					net.reduls.sanmoku.FeatureEx sub_fe = new net.reduls.sanmoku.FeatureEx(sub_e);
					dst.add(sub_fe.reading);
				}

			}
		}
		return dst;
	}

    //segment 原型の配列
	public static ArrayList<String> dummytagAnalyzer(Context con, ArrayList<String> segments){
		ArrayList<String> tags = new ArrayList<String>();
		GetTagFromMysql mysqlTag = new GetTagFromMysql(IPADDR,USER,PASS);
		if(segments != null){
			Log.v(CLASSNAME,"seg:"+segments.size());
			Log.v(CLASSNAME,"param:" + IPADDR + ":" + USER + ":" + PASS);
			
			for(String seg : segments){
				String str = mysqlTag.getTag(seg);
				if(str != null) tags.add(str);
			}
		}
		else{
			Toast.makeText(con, "ERROR", Toast.LENGTH_LONG).show();;
		}

		return tags;
	}

	//dummy
	public static ArrayList<String> tagAnalyzer(Context con, ArrayList<String> segments){
		ArrayList<String> tags = new ArrayList<String>();

		for(String seg : segments){

			if(seg.equals("ノミモノ")
					||seg.equals("ノム")
					||seg.equals("ノド")){
				tags.add("drink");
			}
			else if(seg.equals("オカシ")||
					seg.equals("オヤツ")||
					seg.equals("オナカ")||
					seg.equals("タベル")||
					seg.equals("タベモノ")){
				tags.add("snack");
			}
			else if(seg.equals("オチャ")){
				tags.add("tea");
			}
			else if(seg.equals("コップ")||
					seg.equals("カップ")){
				tags.add("cup");
			}
			else if(seg.equals("アカ")||
					seg.equals("アカイ")||
					seg.equals("アカイロ")){
				tags.add("red");
			}
			else if(seg.equals("アオ")||
					seg.equals("アオイ")||
					seg.equals("アオイロ")){
				tags.add("blue");
			}
			else if(seg.equals("ミドリ")||
					seg.equals("ミドリイロ")){
				tags.add("green");
			}
			else if(seg.equals("シロ")||
					seg.equals("シロイ")||
					seg.equals("シロイロ")){
				tags.add("white");
			}
            else if(seg.equals("クロ")||
                seg.equals("クロイ")||
                seg.equals("クロイロ")){
                tags.add("black");
            }
            else if(seg.equals("ヨム")||
                seg.equals("ホン")){
                tags.add("book");
            }

        }

		return tags;
	}

}

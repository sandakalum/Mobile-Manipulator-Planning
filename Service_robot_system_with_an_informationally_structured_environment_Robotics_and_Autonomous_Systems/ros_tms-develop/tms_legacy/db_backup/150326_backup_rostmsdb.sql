-- MySQL dump 10.13  Distrib 5.6.19, for debian-linux-gnu (x86_64)
--
-- Host: 192.168.4.170    Database: rostmsdb
-- ------------------------------------------------------
-- Server version	5.6.19-0ubuntu0.14.04.1

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `backup_data`
--

DROP TABLE IF EXISTS `backup_data`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `backup_data` (
  `time` datetime(6) NOT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) DEFAULT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  `z` double DEFAULT NULL,
  `rr` double DEFAULT NULL,
  `rp` double DEFAULT NULL,
  `ry` double DEFAULT NULL,
  `offset_x` double DEFAULT NULL,
  `offset_y` double DEFAULT NULL,
  `offset_z` double DEFAULT NULL,
  `joint` varchar(500) DEFAULT NULL,
  `weight` double DEFAULT NULL,
  `rfid` varchar(30) DEFAULT NULL,
  `etcdata` varchar(500) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(500) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` double DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(500) DEFAULT NULL,
  `note` varchar(500) DEFAULT NULL,
  `tag` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`time`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `backup_data`
--

LOCK TABLES `backup_data` WRITE;
/*!40000 ALTER TABLE `backup_data` DISABLE KEYS */;
/*!40000 ALTER TABLE `backup_data` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `data_furniture`
--

DROP TABLE IF EXISTS `data_furniture`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `data_furniture` (
  `time` datetime(6) NOT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) DEFAULT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  `z` double DEFAULT NULL,
  `rr` double DEFAULT NULL,
  `rp` double DEFAULT NULL,
  `ry` double DEFAULT NULL,
  `offset_x` double DEFAULT NULL,
  `offset_y` double DEFAULT NULL,
  `offset_z` double DEFAULT NULL,
  `joint` varchar(500) DEFAULT NULL,
  `weight` double DEFAULT NULL,
  `rfid` varchar(30) DEFAULT NULL,
  `etcdata` varchar(500) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(500) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` double DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(500) DEFAULT NULL,
  `note` varchar(500) DEFAULT NULL,
  `tag` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`time`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `data_furniture`
--

LOCK TABLES `data_furniture` WRITE;
/*!40000 ALTER TABLE `data_furniture` DISABLE KEYS */;
INSERT INTO `data_furniture` VALUES ('2015-03-20 18:07:37.999027','furniture',6019,'wagon',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',3001,0.9,1,'','',''),('2015-03-24 19:12:24.238072','furniture',6001,'wardrobe',9100,5800,0,0,0,0,0,0,1300,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.241311','furniture',6002,'workdesk',7430,5800,0,0,0,0,0,0,1300,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.243746','furniture',6003,'drawer',8400,5700,0,0,0,0,0,0,303,'',0,'','kxp;6700,1900,90',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.245413','furniture',6004,'chair',7500,5400,0,0,0,180,0,0,510,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.247218','furniture',6005,'kitchen',4660,5800,0,0,0,0,0,0,509,'',0,'','',6004,'',0,0,0,'','',''),('2015-03-24 19:12:24.248431','furniture',6006,'meeting_table',1500,1500,0,0,0,0,0,0,360,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.250128','furniture',6007,'meeting_chair1',1500,2400,0,0,0,0,0,0,397.8,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.251654','furniture',6008,'meeting_chair2',700,1800,0,0,0,72,0,0,397.8,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.252976','furniture',6009,'meeting_chair3',1000,800,0,0,0,144,0,0,397.8,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.254108','furniture',6010,'meeting_chair4',2000,800,0,0,0,-144,0,0,397.8,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.256663','furniture',6011,'meeting_chair5',2300,1800,0,0,0,-72,0,0,397.8,'',0,'','smartpal5_1;1900,3400,90;smartpal5_2;1900,3400,90;kxp;2100,3300,90',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.257616','furniture',6012,'partition',6200,2900,0,0,0,-90,0,0,1280,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.259027','furniture',6013,'tv_table',6600,2900,0,0,0,90,0,0,175,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.260760','furniture',6014,'tv_52inch',6700,2900,700,0,0,90,0,0,358.5,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.261918','furniture',6015,'playrecoder',6600,2900,350,0,0,90,0,0,30.5,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.262621','furniture',6016,'sofa',3900,2900,0,0,0,90,0,0,395,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.264222','furniture',6017,'sofa_table',4800,2900,0,0,0,90,0,0,215,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.265059','furniture',6018,'bed',8040,2300,0,0,0,180,0,0,207.5,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.266336','furniture',6019,'wagon',2000,5700,0,0,0,0,305,172.5,350,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.267551','furniture',6020,'shelf',7060,3600,0,0,0,90,0,0,445,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.268416','furniture',6021,'tree',9800,5300,0,0,0,0,358,272,872,'',0,'','',5002,'',0,0,0,'','',''),('2015-03-24 19:12:24.269288','furniture',6022,'tv_multi',10000,2200,1000,0,0,90,0,0,64,'',0,'','',5002,'',0,0,0,'','','');
/*!40000 ALTER TABLE `data_furniture` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `data_object`
--

DROP TABLE IF EXISTS `data_object`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `data_object` (
  `time` datetime(6) NOT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) DEFAULT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  `z` double DEFAULT NULL,
  `rr` double DEFAULT NULL,
  `rp` double DEFAULT NULL,
  `ry` double DEFAULT NULL,
  `offset_x` double DEFAULT NULL,
  `offset_y` double DEFAULT NULL,
  `offset_z` double DEFAULT NULL,
  `joint` varchar(500) DEFAULT NULL,
  `weight` double DEFAULT NULL,
  `rfid` varchar(30) DEFAULT NULL,
  `etcdata` varchar(500) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(500) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` double DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(500) DEFAULT NULL,
  `note` varchar(500) DEFAULT NULL,
  `tag` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`time`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `data_object`
--

LOCK TABLES `data_object` WRITE;
/*!40000 ALTER TABLE `data_object` DISABLE KEYS */;
INSERT INTO `data_object` VALUES ('2014-10-08 14:35:34.302009','object',7002,'chipstar_orange',0,0,0,0,0,0,35,35,70,'',0,'E00401004E180E50','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.302884','object',7003,'chipstar_green',0,0,0,0,0,0,35,35,70,'',0,'E00401004E180E58','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.305622','object',7005,'soukentea_bottle',0,0,0,0,0,0,33,33,81,'',0,'E00401004E180E68','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.306910','object',7006,'cancoffee',0,0,0,0,0,0,26,26,51,'',0,'E00401004E180EA0','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.308294','object',7007,'seasoner_bottle',0,0,0,0,0,0,26,26,94,'',0,'E00401004E180EA8','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.309160','object',7008,'dispenser',0,0,0,0,0,0,40,33,82,'',0,'E00401004E181C88','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.310049','object',7009,'soysauce_bottle_black',1870,4300,1350,0,0,0,32,28,55,'',0,'E00401004E181C87','',6011,'',3005,0,1,'','',''),('2014-10-08 14:35:34.310902','object',7010,'soysauce_bottle_blue',1730,4300,1350,0,0,0,31,28,55,'',0,'E00401004E181C7F','',6011,'',3005,0,1,'','',''),('2014-10-08 14:35:34.312332','object',7011,'soysauce_bottle_white',0,0,0,0,0,0,47,28,44,'',0,'E00401004E181C77','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.313217','object',7012,'pepper_bottle_black',0,0,0,0,0,0,23,23,43,'',0,'E00401004E181C3F','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.314073','object',7013,'pepper_bottle_red',0,0,0,0,0,0,25,25,43,'',0,'E00401004E181C37','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.314942','object',7014,'sake_bottle',0,0,0,0,0,0,35,35,78,'',0,'E00401004E180E47','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.316346','object',7015,'teapot',0,0,0,0,0,0,83,69,42,'',0,'E00401004E180E3F','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.317221','object',7016,'chawan',2100,4300,1350,0,0,0,46,46,50,'',0,'E00401004E180E37','',6011,'',3005,0,1,'','',''),('2014-10-08 14:35:34.318080','object',7017,'teacup1',0,0,0,0,0,0,40,40,28,'',0,'E00401004E1805BD','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.318955','object',7018,'teacup2',2270,4300,1325,0,0,0,42,42,30,'',0,'E00401004E180585','',6011,'',3005,0,1,'','',''),('2014-10-08 14:35:34.320338','object',7019,'cup1',0,0,0,0,0,0,61,47,31,'',0,'E00401004E18057D','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.321208','object',7020,'cup2',0,0,0,0,0,0,53,39,34,'',0,'E00401004E17EF3F','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.322119','object',7021,'mugcup',0,0,0,0,0,0,48,37,36,'',0,'E00401004E17EF37','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.325467','object',7022,'remote',0,0,0,0,0,0,10,26,15,'',0,'E00401004E17EF2F','',0,'',3005,0,0,'','',''),('2014-10-08 14:35:34.327081','object',7023,'book_red',40,50,1420,0,0,0,82,22,123,'',0,'E00401004E17EF27','',6011,'',3005,0,1,'','',''),('2014-10-08 14:35:34.328477','object',7024,'book_blue',0,0,1130,0,0,0,92,17,129,'',0,'E00401004E17EEEF','',6011,'',3005,0,0,'','',''),('2014-10-08 14:35:34.329339','object',7025,'dish',0,0,0,0,0,0,75,75,11,'',0,'E00401004E17EEE7','',0,'',3005,0,0,'','',''),('2015-03-02 11:35:18.742134','object',7001,'chipstar_red',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.743216','object',7002,'chipstar_orange',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.744285','object',7003,'chipstar_green',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.745357','object',7004,'greentea_bottle',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.746428','object',7005,'soukentea_bottle',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.747499','object',7006,'cancoffee',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.748583','object',7007,'seasoner_bottle',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.749677','object',7008,'dispenser',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.750764','object',7009,'soysauce_bottle_black',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.751825','object',7010,'soysauce_bottle_blue',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.752886','object',7011,'soysauce_bottle_white',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.753950','object',7012,'pepper_bottle_black',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.755022','object',7013,'pepper_bottle_red',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.756099','object',7014,'sake_bottle',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.757188','object',7015,'teapot',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.758284','object',7016,'chawan',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.759354','object',7017,'teacup1',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.760423','object',7018,'teacup2',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.761513','object',7019,'cup1',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.762587','object',7020,'cup2',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.763670','object',7021,'mugcup',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.764737','object',7022,'remote',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.765806','object',7023,'book_red',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.766878','object',7024,'book_blue',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 11:35:18.767953','object',7025,'dish',-1,-1,-1,0,0,0,0,0,0,'',0,'','',6010,'',3002,0.8,0,'','',''),('2015-03-02 14:12:31.044214','object',7001,'chipstar_red',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.045278','object',7002,'chipstar_orange',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.046341','object',7003,'chipstar_green',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.048479','object',7005,'soukentea_bottle',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.049548','object',7006,'cancoffee',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.050612','object',7007,'seasoner_bottle',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.051676','object',7008,'dispenser',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.052740','object',7009,'soysauce_bottle_black',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.053804','object',7010,'soysauce_bottle_blue',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.054868','object',7011,'soysauce_bottle_white',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.055934','object',7012,'pepper_bottle_black',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.057000','object',7013,'pepper_bottle_red',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.058065','object',7014,'sake_bottle',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.059129','object',7015,'teapot',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.060193','object',7016,'chawan',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.061256','object',7017,'teacup1',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.062322','object',7018,'teacup2',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.063387','object',7019,'cup1',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.064452','object',7020,'cup2',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.065517','object',7021,'mugcup',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.066581','object',7022,'remote',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.067653','object',7023,'book_red',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.068734','object',7024,'book_blue',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.069799','object',7025,'dish',-1,-1,-1,0,0,0,0,0,0,'',0,'','',2009,'',3018,0.8,0,'','',''),('2015-03-02 14:12:31.926489','object',7004,'greentea_bottle',237,164.695648,800,0,0,0,0,0,0,'',23,'','',2009,'',3018,0.8,1,'','',''),('2015-03-05 16:48:55.422308','object',7001,'chipstar_red',615,30,830,0,0,0,0,0,0,'',0,'','',6011,'',3005,1,1,'','','');
/*!40000 ALTER TABLE `data_object` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `data_person`
--

DROP TABLE IF EXISTS `data_person`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `data_person` (
  `time` datetime(6) NOT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) DEFAULT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  `z` double DEFAULT NULL,
  `rr` double DEFAULT NULL,
  `rp` double DEFAULT NULL,
  `ry` double DEFAULT NULL,
  `offset_x` double DEFAULT NULL,
  `offset_y` double DEFAULT NULL,
  `offset_z` double DEFAULT NULL,
  `joint` varchar(500) DEFAULT NULL,
  `weight` double DEFAULT NULL,
  `rfid` varchar(30) DEFAULT NULL,
  `etcdata` varchar(500) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(500) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` double DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(500) DEFAULT NULL,
  `note` varchar(500) DEFAULT NULL,
  `tag` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`time`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `data_person`
--

LOCK TABLES `data_person` WRITE;
/*!40000 ALTER TABLE `data_person` DISABLE KEYS */;
INSERT INTO `data_person` VALUES ('2014-12-10 18:51:18.000000','person',1001,'person_1',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',3018,0.8,0,'','{\"heartrate\": {\"ros_time\": \"1418237478945482015\", \"val\": \"0\"}}',''),('2015-02-12 17:25:36.176987','person',1001,'person_1',1250,3000,0,0,0,-45,0,0,0,'',0,'','',5001,'',3005,1,1,'','',''),('2015-03-20 18:07:37.998779','person',1001,'person_1',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',3001,0.9,1,'','','');
/*!40000 ALTER TABLE `data_person` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `data_robot`
--

DROP TABLE IF EXISTS `data_robot`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `data_robot` (
  `time` datetime(6) NOT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) DEFAULT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  `z` double DEFAULT NULL,
  `rr` double DEFAULT NULL,
  `rp` double DEFAULT NULL,
  `ry` double DEFAULT NULL,
  `offset_x` double DEFAULT NULL,
  `offset_y` double DEFAULT NULL,
  `offset_z` double DEFAULT NULL,
  `joint` varchar(500) DEFAULT NULL,
  `weight` double DEFAULT NULL,
  `rfid` varchar(30) DEFAULT NULL,
  `etcdata` varchar(500) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(500) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` double DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(500) DEFAULT NULL,
  `note` varchar(500) DEFAULT NULL,
  `tag` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`time`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `data_robot`
--

LOCK TABLES `data_robot` WRITE;
/*!40000 ALTER TABLE `data_robot` DISABLE KEYS */;
INSERT INTO `data_robot` VALUES ('0000-00-00 00:00:00.000000','robot',2007,'wheelchair',1000,0,0,0,0,0,0,0,0,'',0,'','',5001,'',3501,0,0,'','',''),('2014-12-12 20:32:21.968621','robot',2005,'kobuki',0,0,0,90,0,90,0,0,60,'',0,'','',5001,'',3005,1,1,'','',''),('2015-01-19 15:32:59.795162','robot',2007,'wheelchair',0,0,0,0,0,-9.369391,0,0,0,'',0,'','',5001,'',3005,1,1,'','',''),('2015-01-20 16:52:22.582044','robot',2008,'ardrone',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',3001,0.9,1,'','',''),('2015-02-12 17:49:06.297647','robot',2005,'kobuki',0,0,1513.434235,-160.294846,5.409973,-177.654284,0,0,0,'',0,'','',5001,'',3001,0.9,1,'','',''),('2015-02-12 18:29:06.297935','robot',2001,'smartpal4',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',3001,0.9,1,'','',''),('2015-02-14 15:04:48.376596','robot',2002,'smartpal5_1',6801.791496,1638.445776,0,0,0,-135.836576,0,0,0,'-0.248427;0.993797;54.9002;-3.04503;-6.17055;45.09;5.53429;-15.4915;5.32815;-28.6481;-0.457999;-9.98221;0.000499379;-3.15149;0.000181592;-3.15423;1.60186;5.82693e-05',0,'','',5002,'',3003,0.5,1,'','',''),('2015-02-24 16:48:17.041856','robot',2006,'kxp',2149.171143,3017.574707,180,0,0,-146.537674,0,0,180,'0;0;0;0;0;20;20',0,'','',5001,'',3005,1,2,'','',''),('2015-02-24 16:48:17.042466','robot',2011,'kxp2',2149.171143,3017.574707,180,0,0,-146.537674,0,0,180,'0;0;0;0;0;20;20',0,'','',5001,'',3005,1,2,'','',''),('2015-03-05 10:18:49.238109','robot',2002,'smartpal5_1',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',3001,0.9,1,'','',''),('2015-03-05 15:03:07.616047','robot',2003,'smartpal5_2',4960.559377,1047.443398,0,0,0,32.981331,0,0,0,'-0.000336052;-0.000584439;0.175129;-8.64297;0.000953359;0.000499379;-0.000272388;-2.81926;5.38358;-2.96642;-1.92495;-6.11734;-0.00022699;0.294951;-0.00762687;-6.35229;7.37468;-3.80621',0,'','',5002,'',3003,0.5,1,'','',''),('2015-03-05 16:49:40.382508','robot',2002,'smartpal5_1',2149.171143,3017.574707,0,0,0,-146.537674,0,0,0,'-7.08963;28.3585;72.601;-0.802475;-8.16019;59.6282;7.31881;-20.4861;7.04632;-10;0;10;0;0;0;0;0;-10',0,'','',5001,'',3005,1,1,'','',''),('2015-03-05 16:49:40.383008','robot',2003,'smartpal5_2',2149.171143,3017.574707,0,0,0,-146.537674,0,0,0,'0;0;0;-10;0;0;0;0;0;-10;0;10;0;0;0;0;0;-10',0,'','',5001,'',3005,1,1,'','',''),('2015-03-20 18:07:37.998988','robot',2003,'smartpal5_2',5256.191692,1280.754501,1313.520193,0.499481,0.103071,-82.85202,0,0,0,'',0,'','',5001,'',3001,0.9,1,'','',''),('2015-03-20 18:07:37.999048','robot',2007,'wheelchair',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',3001,0.9,1,'','',''),('2015-03-24 19:12:24.212176','robot',2008,'ardrone',4000,2000,1000,0,0,0,0,0,0,'',0,'','',5001,'',0,0,0,'','',''),('2015-03-24 19:12:24.226816','robot',2009,'refrigerator',2850,5800,0,0,0,0,237.5,280.22,560,'',0,'','',5002,'',2009,0,1,'','','');
/*!40000 ALTER TABLE `data_robot` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `data_sensor`
--

DROP TABLE IF EXISTS `data_sensor`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `data_sensor` (
  `time` datetime(6) NOT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) DEFAULT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  `z` double DEFAULT NULL,
  `rr` double DEFAULT NULL,
  `rp` double DEFAULT NULL,
  `ry` double DEFAULT NULL,
  `offset_x` double DEFAULT NULL,
  `offset_y` double DEFAULT NULL,
  `offset_z` double DEFAULT NULL,
  `joint` varchar(500) DEFAULT NULL,
  `weight` double DEFAULT NULL,
  `rfid` varchar(30) DEFAULT NULL,
  `etcdata` varchar(500) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(500) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` double DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(500) DEFAULT NULL,
  `note` varchar(500) DEFAULT NULL,
  `tag` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`time`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `data_sensor`
--

LOCK TABLES `data_sensor` WRITE;
/*!40000 ALTER TABLE `data_sensor` DISABLE KEYS */;
INSERT INTO `data_sensor` VALUES ('2014-07-02 19:22:31.480006','sensor',3014,'portable_sensor_4',300,300,300,0,0,0,0,0,0,'',1,'','1;0;0;0;0;0;0;0;0;0;0;r:0;h:0.0;t:0.0',5001,'',3014,0.7,1,'','',''),('2014-07-02 19:22:33.935769','sensor',3012,'portable_sensor_2',200,200,200,0,0,0,0,0,0,'',0,'','0;0;0;0;0;0;0;0;0;0;0;r:0;h:0.0;t:0.0',5001,'',3012,0.7,1,'','',''),('2014-07-02 19:22:35.173873','sensor',3013,'portable_sensor_3',300,300,300,0,0,0,0,0,0,'',0,'','0;0;0;0;0;0;0;0;0;0;0;r:0;h:0.0;t:0.0',5001,'',3013,0.7,1,'','',''),('2014-10-16 21:02:22.448095','sensor',3016,'m100',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',3001,0.9,1,'','',''),('2015-02-01 00:00:00.000000','sensor',3501,'kalman_filter',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',3501,0,0,'','',''),('2015-02-09 20:16:59.671338','sensor',3011,'portable_sensor_1',100,100,100,0,0,0,0,0,0,'',0,'','00556;1;0;0;0;0;0;38.4;10.6;',5001,'',3011,0.7,1,'','',''),('2015-02-11 20:34:06.932796','sensor',3018,'heartrate_sensor',0,0,0,0,0,0,0,0,0,'',0,'','',1001,'',3018,0.8,0,'','{\"heartrate\": \"81\"}',''),('2015-02-24 17:03:49.536365','sensor',3006,'oculus',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',3001,0.9,1,'','',''),('2015-03-02 16:35:15.683084','sensor',3017,'mindwavemobile',0,0,0,0,0,0,0,0,0,'',0,'','',1001,'',3017,0.5,0,'','{\"meditation\": \"0\", \"attention\": \"0\", \"poor_signal\": \"200\"}',''),('2015-03-20 18:07:37.998896','sensor',3019,'oculus2',5420.402217,-245.934028,726.484264,3.220532,-6.19489,-148.354701,0,0,0,'',0,'','',5001,'',3001,0.9,1,'','','');
/*!40000 ALTER TABLE `data_sensor` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `data_space`
--

DROP TABLE IF EXISTS `data_space`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `data_space` (
  `time` datetime(6) NOT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) DEFAULT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  `z` double DEFAULT NULL,
  `rr` double DEFAULT NULL,
  `rp` double DEFAULT NULL,
  `ry` double DEFAULT NULL,
  `offset_x` double DEFAULT NULL,
  `offset_y` double DEFAULT NULL,
  `offset_z` double DEFAULT NULL,
  `joint` varchar(500) DEFAULT NULL,
  `weight` double DEFAULT NULL,
  `rfid` varchar(30) DEFAULT NULL,
  `etcdata` varchar(500) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(500) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` double DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(500) DEFAULT NULL,
  `note` varchar(500) DEFAULT NULL,
  `tag` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`time`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `data_space`
--

LOCK TABLES `data_space` WRITE;
/*!40000 ALTER TABLE `data_space` DISABLE KEYS */;
INSERT INTO `data_space` VALUES ('2015-03-24 19:12:24.230844','space',5001,'928_room',0,0,0,0,0,0,0,0,200,'',0,'','',4001,'',0,0,0,'','',''),('2015-03-24 19:12:24.232441','space',5002,'928_floor',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,0,0,'','',''),('2015-03-24 19:12:24.234460','space',5003,'928_wall',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,0,0,'','',''),('2015-03-24 19:12:24.236690','space',5004,'928_ceiling',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,0,0,'','','');
/*!40000 ALTER TABLE `data_space` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `data_state`
--

DROP TABLE IF EXISTS `data_state`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `data_state` (
  `time` datetime(6) NOT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) DEFAULT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  `z` double DEFAULT NULL,
  `rr` double DEFAULT NULL,
  `rp` double DEFAULT NULL,
  `ry` double DEFAULT NULL,
  `offset_x` double DEFAULT NULL,
  `offset_y` double DEFAULT NULL,
  `offset_z` double DEFAULT NULL,
  `joint` varchar(500) DEFAULT NULL,
  `weight` double DEFAULT NULL,
  `rfid` varchar(30) DEFAULT NULL,
  `etcdata` varchar(500) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(500) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` double DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(500) DEFAULT NULL,
  `note` varchar(500) DEFAULT NULL,
  `tag` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`time`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `data_state`
--

LOCK TABLES `data_state` WRITE;
/*!40000 ALTER TABLE `data_state` DISABLE KEYS */;
/*!40000 ALTER TABLE `data_state` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `data_structure`
--

DROP TABLE IF EXISTS `data_structure`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `data_structure` (
  `time` datetime(6) NOT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) DEFAULT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  `z` double DEFAULT NULL,
  `rr` double DEFAULT NULL,
  `rp` double DEFAULT NULL,
  `ry` double DEFAULT NULL,
  `offset_x` double DEFAULT NULL,
  `offset_y` double DEFAULT NULL,
  `offset_z` double DEFAULT NULL,
  `joint` varchar(500) DEFAULT NULL,
  `weight` double DEFAULT NULL,
  `rfid` varchar(30) DEFAULT NULL,
  `etcdata` varchar(500) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(500) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` double DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(500) DEFAULT NULL,
  `note` varchar(500) DEFAULT NULL,
  `tag` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`time`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `data_structure`
--

LOCK TABLES `data_structure` WRITE;
/*!40000 ALTER TABLE `data_structure` DISABLE KEYS */;
INSERT INTO `data_structure` VALUES ('2015-03-24 19:12:24.228700','structure',4001,'kyushu_university_west2',0,0,0,0,0,0,0,0,0,'',0,'','',4001,'',0,0,0,'','','');
/*!40000 ALTER TABLE `data_structure` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `history_data`
--

DROP TABLE IF EXISTS `history_data`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `history_data` (
  `time` datetime(6) NOT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) DEFAULT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  `z` double DEFAULT NULL,
  `rr` double DEFAULT NULL,
  `rp` double DEFAULT NULL,
  `ry` double DEFAULT NULL,
  `offset_x` double DEFAULT NULL,
  `offset_y` double DEFAULT NULL,
  `offset_z` double DEFAULT NULL,
  `joint` varchar(500) DEFAULT NULL,
  `weight` double DEFAULT NULL,
  `rfid` varchar(30) DEFAULT NULL,
  `etcdata` varchar(500) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(500) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` double DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(500) DEFAULT NULL,
  `note` varchar(500) DEFAULT NULL,
  `tag` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`time`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `history_data`
--

LOCK TABLES `history_data` WRITE;
/*!40000 ALTER TABLE `history_data` DISABLE KEYS */;
/*!40000 ALTER TABLE `history_data` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `id`
--

DROP TABLE IF EXISTS `id`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `id` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` double DEFAULT NULL,
  `y` double DEFAULT NULL,
  `z` double DEFAULT NULL,
  `rr` double DEFAULT NULL,
  `rp` double DEFAULT NULL,
  `ry` double DEFAULT NULL,
  `offset_x` double DEFAULT NULL,
  `offset_y` double DEFAULT NULL,
  `offset_z` double DEFAULT NULL,
  `joint` varchar(500) DEFAULT NULL,
  `weight` double DEFAULT NULL,
  `rfid` varchar(30) DEFAULT NULL,
  `etcdata` varchar(500) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(500) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` double DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(500) DEFAULT NULL,
  `note` varchar(500) DEFAULT NULL,
  `tag` varchar(500) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `id`
--

LOCK TABLES `id` WRITE;
/*!40000 ALTER TABLE `id` DISABLE KEYS */;
INSERT INTO `id` VALUES ('0000-00-00 00:00:00.000000','IDperson',1000,'------------------------------',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','person',1001,'person_1',4200,5000,900,0,0,90,0,0,0,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','IDrobot',2000,'------------------------------',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','robot',2001,'smartpal4',3000,4000,0,0,0,-90,0,0,0,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','robot',2002,'smartpal5_1',5500,2000,0,0,0,-90,0,0,0,'',0,'','smartpal5_1;5000,1000,-90',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','robot',2003,'smartpal5_2',3000,4000,0,0,0,-90,0,0,0,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','robot',2004,'turtlebot2',0,0,0,0,0,0,0,0,0,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','robot',2005,'kobuki',500,3000,0,90,0,-90,0,0,67,'',0,'','kobuki;4500,700,0',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','robot',2006,'kxp',4000,1000,0,0,0,-90,0,0,180,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','robot',2007,'wheelchair',5000,1000,0,0,0,-90,0,0,0,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','robot',2008,'ardrone',4000,2000,1000,0,0,0,0,0,0,'',0,'','',5001,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','robot',2009,'refrigerator',2850,5800,0,0,0,0,237.5,280.22,560,'',0,'','',5002,'',2009,0,1,'','',''),('0000-00-00 00:00:00.000000','robot',2011,'kxp2',4000,1000,0,0,0,-90,0,0,0,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','IDsensor',3000,'------------------------------',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','sensor',3001,'vicon',0,0,0,0,0,0,0,0,0,'',0,'','',5003,'',0,0.9,1,'','',''),('0000-00-00 00:00:00.000000','sensor',3002,'ics',0,0,0,0,0,0,0,0,0,'',0,'','',5002,'',0,0.8,1,'','',''),('0000-00-00 00:00:00.000000','sensor',3003,'odometry_and_joints',0,0,0,0,0,0,0,0,0,'',0,'','',2002,'',0,0.5,1,'','',''),('0000-00-00 00:00:00.000000','sensor',3004,'reserve',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,1,'','',''),('0000-00-00 00:00:00.000000','sensor',3005,'fake_sensor',0,0,0,0,0,0,0,0,0,'',0,'','',2002,'',0,1,1,'','',''),('0000-00-00 00:00:00.000000','sensor',3006,'oculus',0,0,0,0,0,0,0,0,0,'',0,'','',1001,'',0,0.9,1,'','',''),('0000-00-00 00:00:00.000000','sensor',3011,'portable_sensor_1',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,0.7,0,'','',''),('0000-00-00 00:00:00.000000','sensor',3012,'portable_sensor_2',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,0.7,0,'','',''),('0000-00-00 00:00:00.000000','sensor',3013,'portable_sensor_3',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,0.7,0,'','',''),('0000-00-00 00:00:00.000000','sensor',3014,'portable_sensor_4',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,0.7,0,'','',''),('0000-00-00 00:00:00.000000','sensor',3015,'portable_sensor_5',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,0.7,0,'','',''),('0000-00-00 00:00:00.000000','sensor',3016,'m100',0,0,0,0,0,0,0,0,0,'',0,'','',1001,'',0,0.7,0,'','',''),('0000-00-00 00:00:00.000000','sensor',3017,'brain_wave',0,0,0,0,0,0,0,0,0,'',0,'','',1001,'',0,0.5,0,'','',''),('0000-00-00 00:00:00.000000','sensor',3018,'irs',0,0,0,0,0,0,0,0,0,'',0,'','',5002,'',0,0.8,1,'','',''),('0000-00-00 00:00:00.000000','sensor',3019,'oculus2',0,0,0,0,0,0,0,0,0,'',0,'','',1001,'',0,0.9,1,'','',''),('0000-00-00 00:00:00.000000','sensor',3999,'master',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,1,1,'','',''),('0000-00-00 00:00:00.000000','IDstructure',4000,'------------------------------',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','structure',4001,'kyushu_university_west2',0,0,0,0,0,0,0,0,0,'',0,'','',4001,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','IDspace',5000,'------------------------------',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','space',5001,'928_room',0,0,0,0,0,0,0,0,200,'',0,'','',4001,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','space',5002,'928_floor',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','space',5003,'928_wall',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','space',5004,'928_ceiling',0,0,0,0,0,0,0,0,0,'',0,'','',5001,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','space',5005,'928_corridor',0,0,0,0,0,90,10260,4600,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','IDfurniture',6000,'------------------------------',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6001,'wardrobe',9100,5800,0,0,0,0,0,0,1300,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6002,'workdesk',7430,5800,0,0,0,0,0,0,1300,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6003,'drawer',8400,5700,0,0,0,0,0,0,303,'',0,'','kxp;6700,1900,90',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6004,'chair',7500,5400,0,0,0,180,0,0,510,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6005,'kitchen',4660,5800,0,0,0,0,0,0,509,'',0,'','',6004,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6006,'meeting_table',1500,1500,0,0,0,0,0,0,360,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6007,'meeting_chair1',1500,2400,0,0,0,0,0,0,397.8,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6008,'meeting_chair2',700,1800,0,0,0,72,0,0,397.8,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6009,'meeting_chair3',1000,800,0,0,0,144,0,0,397.8,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6010,'meeting_chair4',2000,800,0,0,0,-144,0,0,397.8,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6011,'meeting_chair5',2300,1800,0,0,0,-72,0,0,397.8,'',0,'','smartpal5_1;1900,3400,90;smartpal5_2;1900,3400,90;kxp;2100,3300,90',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6012,'partition',6200,2900,0,0,0,-90,0,0,1280,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6013,'tv_table',6600,2900,0,0,0,90,0,0,175,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6014,'tv_52inch',6700,2900,700,0,0,90,0,0,358.5,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6015,'playrecoder',6600,2900,350,0,0,90,0,0,30.5,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6016,'sofa',3900,2900,0,0,0,90,0,0,395,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6017,'sofa_table',4800,2900,0,0,0,90,0,0,215,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6018,'bed',8040,2300,0,0,0,180,0,0,207.5,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6019,'wagon',2000,5700,0,0,0,0,305,172.5,350,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6020,'shelf',7060,3600,0,0,0,90,0,0,445,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6021,'tree',9800,5300,0,0,0,0,358,272,872,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','furniture',6022,'tv_multi',10000,2200,1000,0,0,90,0,0,64,'',0,'','',5002,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','IDobject',7000,'------------------------------',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','object',7001,'chipstar_red',0,0,0,0,0,0,35,35,70,'',0,'E00401004E17F97A','',0,'',0,0,0,'','','snack;red'),('0000-00-00 00:00:00.000000','object',7002,'chipstar_orange',0,0,0,0,0,0,35,35,70,'',0,'E00401004E180E50','',0,'',0,0,0,'','','snack;orange'),('0000-00-00 00:00:00.000000','object',7003,'chipstar_green',0,0,0,0,0,0,35,35,70,'',0,'E00401004E180E58','',0,'',0,0,0,'','','snack;green'),('0000-00-00 00:00:00.000000','object',7004,'greentea_bottle',0,0,0,0,0,0,33,33,83,'',0,'E00401004E180E60','',6010,'',0,0,0,'','','drink;tea;water'),('0000-00-00 00:00:00.000000','object',7005,'soukentea_bottle',0,0,0,0,0,0,33,33,81,'',0,'E00401004E180E68','',0,'',0,0,0,'','','drink;tea;water'),('0000-00-00 00:00:00.000000','object',7006,'cancoffee',0,0,0,0,0,0,26,26,51,'',0,'E00401004E180EA0','',0,'',0,0,0,'','','drink;coffee;water'),('0000-00-00 00:00:00.000000','object',7007,'seasoner_bottle',0,0,0,0,0,0,26,26,94,'',0,'E00401004E180EA8','',0,'',0,0,0,'','','seasoning;white'),('0000-00-00 00:00:00.000000','object',7008,'dispenser',0,0,0,0,0,0,40,33,82,'',0,'E00401004E181C88','',0,'',0,0,0,'','','seasoning;white'),('0000-00-00 00:00:00.000000','object',7009,'soysauce_bottle_black',0,0,0,0,0,0,32,28,55,'',0,'E00401004E181C87','',0,'',0,0,0,'','','seasoning;black'),('0000-00-00 00:00:00.000000','object',7010,'soysauce_bottle_blue',0,0,0,0,0,0,31,28,55,'',0,'E00401004E181C7F','',0,'',0,0,0,'','','seasoning;blue'),('0000-00-00 00:00:00.000000','object',7011,'soysauce_bottle_white',0,0,0,0,0,0,47,28,44,'',0,'E00401004E181C77','',0,'',0,0,0,'','','seasoning;white'),('0000-00-00 00:00:00.000000','object',7012,'pepper_bottle_black',0,0,0,0,0,0,23,23,43,'',0,'E00401004E181C3F','',0,'',0,0,0,'','','seasoning;black'),('0000-00-00 00:00:00.000000','object',7013,'pepper_bottle_red',0,0,0,0,0,0,25,25,43,'',0,'E00401004E181C37','',0,'',0,0,0,'','','seasoning;red'),('0000-00-00 00:00:00.000000','object',7014,'sake_bottle',0,0,0,0,0,0,35,35,78,'',0,'E00401004E180E47','',0,'',0,0,0,'','','drink;alcoholic'),('0000-00-00 00:00:00.000000','object',7015,'teapot',0,0,0,0,0,0,83,69,42,'',0,'E00401004E180E3F','',0,'',0,0,0,'','','dish;orange'),('0000-00-00 00:00:00.000000','object',7016,'chawan',0,0,0,0,0,0,46,46,50,'',0,'E00401004E180E37','',0,'',0,0,0,'','','dish;white'),('0000-00-00 00:00:00.000000','object',7017,'teacup1',0,0,0,0,0,0,40,40,28,'',0,'E00401004E1805BD','',0,'',0,0,0,'','','cup;blue'),('0000-00-00 00:00:00.000000','object',7018,'teacup2',0,0,0,0,0,0,42,42,30,'',0,'E00401004E180585','',0,'',0,0,0,'','','cup;blue'),('0000-00-00 00:00:00.000000','object',7019,'cup1',0,0,0,0,0,0,61,47,31,'',0,'E00401004E18057D','',0,'',0,0,0,'','','cup;white'),('0000-00-00 00:00:00.000000','object',7020,'cup2',0,0,0,0,0,0,53,39,34,'',0,'E00401004E17EF3F','',0,'',0,0,0,'','','cup;white'),('0000-00-00 00:00:00.000000','object',7021,'mugcup',0,0,0,0,0,0,48,37,36,'',0,'E00401004E17EF37','',0,'',0,0,0,'','','cup;red'),('0000-00-00 00:00:00.000000','object',7022,'remote',0,0,0,0,0,0,10,26,15,'',0,'E00401004E17EF2F','',0,'',0,0,0,'','','remote'),('0000-00-00 00:00:00.000000','object',7023,'book_red',0,0,0,0,0,0,82,22,123,'',0,'E00401004E17EF27','',0,'',0,0,0,'','','book;red'),('0000-00-00 00:00:00.000000','object',7024,'book_blue',0,0,0,0,0,0,92,17,129,'',0,'E00401004E17EEEF','',0,'',0,0,0,'','','book;blue'),('0000-00-00 00:00:00.000000','object',7025,'dish',0,0,0,0,0,0,75,75,11,'',0,'E00401004E17EEE7','',0,'',0,0,0,'','','dish;white'),('0000-00-00 00:00:00.000000','object',7026,'watering_pot',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','','pot;water'),('0000-00-00 00:00:00.000000','IDtask',8000,'------------------------------',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','task',8001,'get_object',0,0,0,0,0,0,0,0,0,'',0,'','9001$oid 9002$oid + 9003$uid +',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','task',8002,'patrol',0,0,0,0,0,0,0,0,0,'',0,'','9001$rid 9006$oid 9007$oid | +',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','task',8003,'test_task',0,0,0,0,0,0,0,0,0,'',0,'','9006$oid 9007$oid |',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','task',8004,'generate_script_test',0,0,0,0,0,0,0,0,0,'',0,'','9001$-1$5500$2000$-90 9001$-1$2000$3000$90 + 9001$-1$5500$2000$-90 +',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','IDsubtask',9000,'------------------------------',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','subtask',9001,'move',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','subtask',9002,'grasp',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','subtask',9003,'give',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','subtask',9004,'open',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','subtask',9005,'close',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','subtask',9006,'random_move',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','subtask',9007,'sensing',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','IDstate',10000,'------------------------------',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','IDetc',20000,'------------------------------',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','etc',20001,'blink_arrow',0,0,0,0,0,0,50,50,125,'',0,'','',0,'',0,0,0,'','',''),('0000-00-00 00:00:00.000000','etc',20002,'person_marker',0,0,0,0,0,0,0,0,0,'',0,'','',0,'',0,0,0,'','','');
/*!40000 ALTER TABLE `id` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2015-03-26 12:21:30

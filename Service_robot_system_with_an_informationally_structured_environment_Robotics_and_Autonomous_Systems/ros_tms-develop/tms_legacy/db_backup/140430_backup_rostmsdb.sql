-- MySQL dump 10.13  Distrib 5.5.37, for debian-linux-gnu (x86_64)
--
-- Host: 192.168.4.170    Database: rostmsdb
-- ------------------------------------------------------
-- Server version	5.6.16-1~exp1

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
-- Table structure for table `backup`
--

DROP TABLE IF EXISTS `backup`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `backup` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `backup`
--

LOCK TABLES `backup` WRITE;
/*!40000 ALTER TABLE `backup` DISABLE KEYS */;
/*!40000 ALTER TABLE `backup` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `building`
--

DROP TABLE IF EXISTS `building`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `building` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `building`
--

LOCK TABLES `building` WRITE;
/*!40000 ALTER TABLE `building` DISABLE KEYS */;
/*!40000 ALTER TABLE `building` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `data`
--

DROP TABLE IF EXISTS `data`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `data` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `data`
--

LOCK TABLES `data` WRITE;
/*!40000 ALTER TABLE `data` DISABLE KEYS */;
/*!40000 ALTER TABLE `data` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `furniture`
--

DROP TABLE IF EXISTS `furniture`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `furniture` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `furniture`
--

LOCK TABLES `furniture` WRITE;
/*!40000 ALTER TABLE `furniture` DISABLE KEYS */;
/*!40000 ALTER TABLE `furniture` ENABLE KEYS */;
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
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `id`
--

LOCK TABLES `id` WRITE;
/*!40000 ALTER TABLE `id` DISABLE KEYS */;
INSERT INTO `id` VALUES (NULL,'person',1000,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'person',1001,'person1',0,0,0,0,0,0,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'robot',2000,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'robot',2001,'smartpal4',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'robot',2002,'smartpal5_1',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'robot',2003,'smartpal5_2',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'robot',2004,'turtlebot2',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'robot',2005,'kobuki',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'robot',2006,'kxp',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'robot',2007,'wcr',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'sensor',3000,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'structure',4000,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'space',5000,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'space',5001,'floor928',0,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'space',5002,'wall928',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6000,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6001,'big_sofa',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6002,'mini_sofa',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6003,'small_table',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6004,'tv_table',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6005,'tv',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6006,'partition1050x900',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6007,'partition1050x700',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6008,'partition1050x900',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6009,'bed',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6010,'shelf1',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6011,'big_shelf',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6012,'desk1',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6013,'chair1',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6014,'table',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'furniture',6015,'chair1',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7000,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7001,'chipstar_red',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7002,'chipstar_orange',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7003,'chipstar_green',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7004,'greentea_bottle',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7005,'soukentea_bottle',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7006,'cancoffee',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7007,'seasoner_bottle',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7008,'dispenser',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7009,'soysauce_bottle_black',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7010,'soysauce_bottle_blue',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7011,'soysauce_bottle_white',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7012,'pepper_bottle_black',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7013,'pepper_bottle_red',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7014,'sake_bottle',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7015,'teapot',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7016,'chawan',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7017,'teacup1',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7018,'teacup2',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7019,'cup1',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7020,'cup2',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7021,'mugcup',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7022,'remote',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7023,'book_red',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7024,'book_blue',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'object',7025,'dish',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'task',8000,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'subtask',9000,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),(NULL,'state',10000,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL);
/*!40000 ALTER TABLE `id` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `object`
--

DROP TABLE IF EXISTS `object`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `object` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `object`
--

LOCK TABLES `object` WRITE;
/*!40000 ALTER TABLE `object` DISABLE KEYS */;
/*!40000 ALTER TABLE `object` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `person`
--

DROP TABLE IF EXISTS `person`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `person` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `person`
--

LOCK TABLES `person` WRITE;
/*!40000 ALTER TABLE `person` DISABLE KEYS */;
/*!40000 ALTER TABLE `person` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `robot`
--

DROP TABLE IF EXISTS `robot`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `robot` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `robot`
--

LOCK TABLES `robot` WRITE;
/*!40000 ALTER TABLE `robot` DISABLE KEYS */;
/*!40000 ALTER TABLE `robot` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `sensor`
--

DROP TABLE IF EXISTS `sensor`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `sensor` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `sensor`
--

LOCK TABLES `sensor` WRITE;
/*!40000 ALTER TABLE `sensor` DISABLE KEYS */;
/*!40000 ALTER TABLE `sensor` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `space`
--

DROP TABLE IF EXISTS `space`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `space` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `space`
--

LOCK TABLES `space` WRITE;
/*!40000 ALTER TABLE `space` DISABLE KEYS */;
/*!40000 ALTER TABLE `space` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `state`
--

DROP TABLE IF EXISTS `state`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `state` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `state`
--

LOCK TABLES `state` WRITE;
/*!40000 ALTER TABLE `state` DISABLE KEYS */;
/*!40000 ALTER TABLE `state` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `subtask`
--

DROP TABLE IF EXISTS `subtask`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `subtask` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `subtask`
--

LOCK TABLES `subtask` WRITE;
/*!40000 ALTER TABLE `subtask` DISABLE KEYS */;
/*!40000 ALTER TABLE `subtask` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `task`
--

DROP TABLE IF EXISTS `task`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `task` (
  `time` datetime(6) DEFAULT NULL,
  `type` varchar(30) DEFAULT NULL,
  `id` int(11) NOT NULL,
  `name` varchar(30) DEFAULT NULL,
  `x` float DEFAULT NULL,
  `y` float DEFAULT NULL,
  `z` float DEFAULT NULL,
  `rr` float DEFAULT NULL,
  `rp` float DEFAULT NULL,
  `ry` float DEFAULT NULL,
  `offset_h` float DEFAULT NULL,
  `offset_w` float DEFAULT NULL,
  `joint` varchar(100) DEFAULT NULL,
  `weight` float DEFAULT NULL,
  `rfid` int(11) DEFAULT NULL,
  `etcdata` varchar(100) DEFAULT NULL,
  `place` int(11) DEFAULT NULL,
  `extfile` varchar(100) DEFAULT NULL,
  `sensor` int(11) DEFAULT NULL,
  `probability` float DEFAULT NULL,
  `state` int(11) DEFAULT NULL,
  `task` varchar(100) DEFAULT NULL,
  `note` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task`
--

LOCK TABLES `task` WRITE;
/*!40000 ALTER TABLE `task` DISABLE KEYS */;
/*!40000 ALTER TABLE `task` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2014-04-30 20:29:42

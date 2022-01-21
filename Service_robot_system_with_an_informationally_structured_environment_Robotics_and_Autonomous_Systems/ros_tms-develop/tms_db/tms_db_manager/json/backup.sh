#!/bin/sh
roscd tms_db_manager/json/
mongoexport --db rostmsdb --collection default --out default.json
mongoexport --db rostmsdb --collection now --out now.json
DATE=`date '+%Y%m%d'`
tar -cf $DATE"_db_backup.tar" default.json now.json

# tms_nw_api module
* 上位層のtmsから送られてきたリクエストを、自環境が実行可能かどうかを判断し、実行可能であれば'user_id', 'robot_id', 'object_id', 'place_id', 'task_id'をリクエスト元のIPアドレスと紐付けて保持、一致するIPアドレスからのリクエストがあれば返答としてそれらを受け渡す。
# OMRON SENTECHカメラ用ROS２ノード

## 1. 概要
stcamera_ros2パッケージはオムロン センテック株式会社が提供するパッケージで、SentechSDKに対応しているカメラをROS2で使用するためのものです。
ROS2のトピックやサービスを使用して、カメラから画像の取得やカメラの設定、トリガー生成等を行うことができます。
*GenICam GenApi*ノードとROSにあるノードとは全く別の物であることにご注意下さい。 各GenICamGenApiノードは、汎用的なインターフェースを使用して、カメラの情報取得や設定に使用されます。GenICam GenApiの詳細は[GenICam GenApiドキュメント（英語）]をご参照下さい。

## 2. インストール
本パッケージはUbuntu 22.04 64ビット上のROS2-Humble Hawksbillで動作確認を行っております。
本パッケージはSentechSDK(v1.2.1以降)を利用するため、本パッケージをインストールする前に、予めSentechSDKをインストールしてください。SetenchSDKを初めて使用される場合は、SentechSDK付属のドキュメントをご覧の上、SentechSDKのViewer(StViewer)でカメラからの画像が取得できることを確認しておくと、後の作業がスムーズに行えます。また、StViewerを使用して予めカメラの設定を変更し、UserSetSave/UserSetDefaultを使用してカメラにその設定を保存しておくことで、ROS2でのカメラ設定が不要になる場合があります。SentechSDKは、下記のURLからダウンロード可能です。

https://sentech.co.jp/data/

SentechSDKのインストール方法や使用方法の詳細については、SentechSDKに付属の資料をご参照ください。
SentechSDKを使用するための環境変数が未設定の場合は、下記のようなコマンドを実行して下さい（SentechSDKを/opt/sentechにインストールした場合）。

 ``$ source /opt/sentech/.stprofile``

ROS2の環境設定が行われていない場合は、下記のコマンドを実行してください。

 ``$ source /opt/ros/humble/setup.bash``

ワークスペース(例:~/dev_ws)を作成し、カレントディレクトリを移動します。

 ``$ mkdir -p ~/dev_ws/src``

 ``$ cd ~/dev_ws/src``

stcamera_ros2プロジェクトをクローンします。

 ``$ git clone https://github.com/ose-support-ros/stcamera_ros2.git -b humble``

 依存関係をチェックし、必要なパッケージをインストールします。

 ``$ cd ~/dev_ws``

 ``$ rosdep install -i --from-path src --rosdistro humble -y``

 ビルドします。

 ``$ colcon build``

## 3. StCameraNode
パッケージにあるStCameraNodeを生成すると、パラメータ**camera_to_connect**に応じて指定されたカメラからの画像取得が開始されます。
StCameraNodeは、下記のいずれかのコマンドで生成できます。

``$ source install/setup.bash``

``$ ros2 launch stcamera_launch stcamera_launch.py``

又は

``$ source install/setup.bash``

``$ ros2 component standalone --node-name 'stcameras' --node-namespace '/stcamera_launch' stcamera_components stcamera::StCameraNode``

又は

``$ source install/setup.bash``

``$ ros2 run rclcpp_components component_container``

``$ ros2 component load /ComponentManager  -n 'stcameras' --node-namespace '/stcamera_launch' stcamera_components stcamera::StCameraNode``

2番目、3番目の方法で記載されているノード名やネームスペースの指定はオプションですが、上記のように指定しておくことで、サンプルプログラムを動作させることができるようになります。
StCameraNodeをstcamera_launchファイルスクリプトで起動するとカメラ接続パラメータがstcamera_node.yamlで簡単に設定できます。 パラメータ設定の詳細は次の章を参照下さい。

### 3.1. ノードパラメータ
パラメータの値は大文字と小文字を区別します。設定できるパラメータは下記の通りです。

  * **camera_to_connect** : ROS2で使用するカメラを指定します。このパラメーターは、StCameraNodeが実行される前に設定する必要があります。 実行中にこのパラメーターを変更しても、その変更は反映されません。設定が空の場合、最初に検出されたカメラが使用されます。"all"が指定されている場合は、検出された全てのカメラが使用されます。CAMERA_ID又はCAMERA_MODEL(SERIAL)が指定されている場合、指定したカメラのみ使用されます。
  
  例

    * ``camera_to_connect:[]``: 最初に検出されたカメラが使用されます。
    * ``camera_to_connect:["all"]``: 検出された全てのカメラが使用されます。
    * ``camera_to_connect:["00:11:1c:f6:yy:xx","STC-MCS510U3V(00XXYY0)"]``: MACアドレス「00:11:1c:f6:yy:xx」のGigEVisionカメラとシリアル番号「00XXYY0」のUSB3Visionカメラ(STC-MCS510U3V)のみが使用されます。
    * ``camera_to_connect:["14210003XXYY"]``: IDが「14210003XXYY」のカメラが使用されます。

### 3.2. カメラの名前空間（ネームスペース）
StCameraNodeはカメラごとにトピックとサービスを宣言します。個々のカメラのトピックおよびサービスへアクセスする際に使用される各カメラのネームスペースは、下記のルールで自動的に生成されます。

* **camera_to_connect**が空または["all"]の場合、ネームスペースの形式は *dev\_{CAMERA\_ID}*になります。*{CAMERA\_ID}* はカメラのID。接続されたカメラIDが「14210003xxYY」の場合、ネームスペースは「*dev_14210003xxYY*」になります. GigEVisionのMACアドレスなどのカメラIDの英数字以外の文字は、アンダースコアに置き換えられます。
* **camera_to_connect**がCAMERA\_MODEL(SERIAL)またはCAMERA\_IDの場合、ネームスペースの形式は*dev\_{CAMERA\_MODEL\_SERIAL\_}*または *dev\_{CAMERA\_ID}*になります。*{CAMERA\_MODEL\_SERIAL\_}*はCAMERA_MODEL(SERIAL)からの書き換えられた文字列です. *{CAMERA\_ID}* はカメラIDです。英数字以外の文字は、アンダースコアに置き換えられます。

### 3.3. トピック
StCameraNodeにより発行されるトピックは下記の通りです。

トピック                           | 説明
---------------------------------- | ----------------------------------------------
**device_connection**              | 接続または切断したときに発行されます。
*{dev\_CAMERA-NS}*/**event**       | カメライベントを登録し、登録されたイベントが発生したときに発行されます。
*{dev\_CAMERA-NS}*/**chunk**       | チャンクデータを取得したときに発行されます。
*{dev\_CAMERA-NS}*/**image_raw\*** | 画像データを取得したときに発行されます（ROSのimage_transportに基づくトピックです）。
*{dev\_CAMERA-NS}*/**camera_info** | ROSのimage_transportに基づくトピックです。

### 3.4. サービス
StCameraNodeが提供するサービスは下記の通りです。

（下記に記載するGenICamノードはROSのノードとは全く別の物です）

サービス                           | 説明
---------------------------------- | ----------------------------------------------
**get_sdk_info**                   | SentechSDKバージョンとGenTLプロデューサの情報を取得します。
**get_module_list**                | GenTLモジュール名のリストを取得します（*System*、*Interface*、*LocalDevice*、*RemoteDevice*、*DataStream*）。 モジュール名は特定のGenICamノードにアクセスする際に引数として渡す必要があります。例えば、read_nodeまたはwrite_nodeサービス経由でカメラの*Gain*にアクセスするには、モジュール名にRemoteDeviceを、GenICamノード名にGainを指定する必要があります。
**get_device_list**                | 検出されたデバイスのリストを取得します。
*{dev\_CAMERA-NS}*/**get_image_acquisition_status** | 画像取得のステータスを取得します。
*{dev\_CAMERA-NS}*/**enable_image_acquisition** | 画像取得を開始(有効)または停止(無効)します（StCameraNode生成時またはカメラ接続時に、自動で有効になります）。
*{dev\_CAMERA-NS}*/**get_event_acquisition_status_list** | 全GenTLモジュールのイベント取得のステータスを取得します。
*{dev\_CAMERA-NS}*/**enable_event_acquisition** | 指定されたGenTLモジュールのイベント取得を開始(有効)または停止(無効)します。
*{dev\_CAMERA-NS}*/**get_event_node_status_list** | 指定されたGenTLモジュールのGenICamノードイベント取得のステータスを取得します。
*{dev\_CAMERA-NS}*/**enable_event_node** | 指定されたGenTLモジュールのGenICamノードのイベントコールバックを有効または無効にします。
*{dev\_CAMERA-NS}*/**get_chunk_list** | カメラがサポートするチャンクのリストとステータスを取得します。利用する前に、画像取り込みを停止させる必要があります。
*{dev\_CAMERA-NS}*/**enable_chunk** | カメラのチャンク出力を有効または無効に設定します。
*{dev\_CAMERA-NS}*/**get_trigger_list** | カメラがサポートしているトリガーのリストを取得します。
*{dev\_CAMERA-NS}*/**enable_trigger** | 指定されたトリガのOn/Offやトリガー源などの設定を行います。
*{dev\_CAMERA-NS}*/**send_soft_trigger** | カメラへソフトウェアトリガーを送信します。
*{dev\_CAMERA-NS}*/**execute_node** | 指定されたGenICamノード(ICommandインターフェースタイプ)を実行します。
*{dev\_CAMERA-NS}*/**get_genicam_node_info** | 指定されたGenICamノードの情報を取得します。
*{dev\_CAMERA-NS}*/**read_node** | 指定されたGenICamノードの値を文字列として読み取ります。GenICamノードのインターフェースタイプがIInteger, IFloat, IBoolean, IString, IEnumerationの時のみ使用可能です。
*{dev\_CAMERA-NS}*/**write_node** | 指定されたGenICamノードの値を指定した文字列で更新します。GenICamノードのインターフェースタイプがIInteger, IFloat, IBoolean, IString, IEnumerationの時のみ使用可能です。
*{dev\_CAMERA-NS}*/**read_node_bool** | 指定されたGenICamノード(IBooleanインターフェースタイプ)の値を読み取ります。
*{dev\_CAMERA-NS}*/**write_node_bool** | 指定されたGenICamノード(IBooleanインターフェースタイプ)に値を書き込みます。
*{dev\_CAMERA-NS}*/**get_enum_list** | 指定されたGenICamノード(IEnumerationインターフェースタイプ)の列挙子のリストを取得します。
*{dev\_CAMERA-NS}*/**read_node_enum** | 指定されたGenICamノード(IEnumerationインターフェースタイプ)の値を読み込みます。
*{dev\_CAMERA-NS}*/**write_node_enum_int** | 指定されたGenICamノード(IEnumerationインターフェースタイプ)に列挙子の整数値を書き込みます。
*{dev\_CAMERA-NS}*/**write_node_enum_str** | 指定されたGenICamノード(IEnumerationインターフェースタイプ)に列挙子の文字列を書き込みます。
*{dev\_CAMERA-NS}*/**read_node_int** | 指定されたGenICamノード(IIntegerインターフェースタイプ)の値を読み取ります。
*{dev\_CAMERA-NS}*/**write_node_int** | 指定されたGenICamノード(IIntegerインターフェースタイプ)に値を書き込みます。
*{dev\_CAMERA-NS}*/**read_node_float** | 指定されたGenICamノード(IFloatインターフェースタイプ)の値を読み取ります。
*{dev\_CAMERA-NS}*/**write_node_float** | 指定されたGenICamノード(IFloatインターフェースタイプ)に値を書き込みます。
*{dev\_CAMERA-NS}*/**read_node_string** | 指定されたGenICamノード(IStringインターフェースタイプ)の値を読み取ります。
*{dev\_CAMERA-NS}*/**write_node_string** | 指定されたGenICamノード(IStringインターフェースタイプ)に値を書き込みます。
*{dev\_CAMERA-NS}*/**read_node_port** | 指定されたGenICamノード(IPortインターフェースタイプ)を使用して指定したアドレスの値を読み取ります。
*{dev\_CAMERA-NS}*/**write_node_port** | 指定されたGenICamノード(IPortインターフェースタイプ)を使用して指定したアドレスに値を書き込みます。
*{dev\_CAMERA-NS}*/**read_node_register_info** | 指定されたGenICamノード(IRegisterインターフェースタイプ)の情報を読み取ります。
*{dev\_CAMERA-NS}*/**read_node_register** | 指定されたGenICamノード(IRegisterインターフェースタイプ)の値を読み取ります。
*{dev\_CAMERA-NS}*/**write_node_register** | 指定されたGenICamノード(IRegisterインターフェースタイプ)に値を書き込みます。
*{dev\_CAMERA-NS}*/**set_camera_info** | ROSのimage_transportに基づくサービスです。

注意

* 画像取得中にはアクセスできないGenICamノードがあります（エラー発生）。その際には画像の取得を停止にしてから再度アクセスして下さい。サービスコール**enable_image_acquisition**で画像取得停止ができます。例：<br />

``$ ros2 service call /stcamera_launch/dev_CAMERA-NS/enable_image_acquisition stcamera_msgs/srv/EnableImageAcquisition "{value: false}"``

<pre>
waiting for service to become available...
requester: making request: stcamera_msgs.srv.EnableImageAcquisition_Request(value=False)

response:
stcamera_msgs.srv.EnableImageAcquisition_Response()
</pre>

* IntegerタイプのGenICamノードは設定できる値に制限がある場合があります。下記の設定値ではincrementが16になっているため、増減させる際の値がその倍数になっていない場合にはエラーが発生します。<br />

``$ ros2 service call /stcamera_launch/dev_CAMERA-NS/get_genicam_node_info stcamera_msgs/srv/GetGenICamNodeInfo "{genicam_module: 'RemoteDevice', genicam_node: 'Width'}"``

<pre>
requester: making request: stcamera_msgs.srv.GetGenICamNodeInfo_Request(genicam_module='RemoteDevice', genicam_node='Width')

response:
stcamera_msgs.srv.GetGenICamNodeInfo_Response(name='Width', description='Width of the image provided by the device (in pixels).', name_space='Standard', interface_type='IInteger', access_mode='Read Only', is_cachable='Yes', visibility='Beginner', caching_mode='Write to register, write to cache on read', is_streamable=True, enum_value_str_list=[], enum_value_int_list=[], current_value='2448', min_value='64', max_value='2448', increment='16', unit='', child_node_list=[])
</pre>

* 使用中に変更する必要のないカメラ設定については、予めSentechSDK付属のStViewerで設定を行い、カメラへ保存(UserSetSave)し、カメラの電源投入時に反映されるように設定(UserSetDefault)しておくことで、毎回設定する手間を省くことができます。


### 3.5. エラーコード
 コード | 説明
 ---: | ---
 < 0 | GenTLエラーコード。[GenICam GenTLドキュメント（英語）]をご参照下さい。
 0 | エラー無.
 30000 | GenICamのジェネリック例外が発生しました。
 30001 | GenICamモジュール名が無効です。
 30002 | GenICamノードが無効またはアクセス不能です。
 30003 | イベントがすでに有効です。
 30004 | イベントがすでに無効です。
 30005 | 画像取得がすでに有効です。
 30006 | 画像取得がすでに無効です。
 30007 | チャンクがサポートされていません。
 30008 | チャンク名が無効です。
 30009 | トリガーがサポートされていません。
 30010 | トリガー名が無効です。
 30011 | イベントがサポートされていません。
 30012 | イベント名が無効です。

## 4. 使用法
### 4.1. rqt_image_viewで取得画像を表示し、コマンドラインからカメラのGain設定を変更する

* StCameraNodeを生成します。

``$ cd ~/dev_ws``

``$ source install/setup.bash``

``$ ros2 launch stcamera_launch stcamera_launch.py``

* 現在発行されているトピックの名前を確認します。

``$ ros2 topic list``

* rqt_image_viewを実行し、「/xxxx/xxxx/image_raw」を選択して映像を確認します。<br />

``$ ros2 run rqt_image_view rqt_image_view``

** カメラがトリガーモードになっていると、トリガーが生成されるまで画像が取得できません。一旦ノードを破棄し、StViewer等でフリーランモードへ切り替えてから再度ご確認ください。

* 現在発行されているサービスおよびその型は、下記のようなコマンドで確認できます。

``$ ros2 service list -t``

* 型の詳細については、下記のようなコマンドで確認できます。

``ros2 interface show stcamera_msgs/srv/GetGenICamNodeInfo``

* GenICamノード”Gain”の情報を取得します。

``$ ros2 service call /stcamera_launch/dev_142100000000/get_genicam_node_info stcamera_msgs/srv/GetGenICamNodeInfo '{genicam_module: "RemoteDevice", genicam_node: "Gain"}'``

<pre>
waiting for service to become available...
requester: making request: stcamera_msgs.srv.GetGenICamNodeInfo_Request(genicam_module='RemoteDevice', genicam_node='Gain')

response:
stcamera_msgs.srv.GetGenICamNodeInfo_Response(error_info=stcamera_msgs.msg.ErrorInfo(error_code=0, description=''), name='Gain', description='Controls the selected gain as an absolute physical value. This is an amplification factor applied to the video signal.', tool_tip='Controls the selected gain as an absolute physical value.', display_name='Gain', name_space='Standard', interface_type='IFloat', access_mode='Read and Write', is_cachable='Yes', visibility='Beginner', caching_mode='Does not use cache', polling_time=-1, is_streamable=True, is_implemented=True, is_available=True, is_readable=True, is_writable=True, is_feature=True, enum_value_str_list=[], enum_value_int_list=[], current_value='0', min_value='0.000000', max_value='192.000000', increment='', unit='', child_node_list=[])
</pre>

* GenICamノード”Gain”(IFloat型)の値を読み込みます。<br />

``$ ros2 service call /stcamera_launch/dev_142100000000/read_node_float stcamera_msgs/srv/ReadNodeFloat '{genicam_module: "RemoteDevice", genicam_node: "Gain"}'``<br /> 

<pre>
waiting for service to become available...
requester: making request: stcamera_msgs.srv.ReadNodeFloat_Request(genicam_module='RemoteDevice', genicam_node='Gain')

response:
stcamera_msgs.srv.ReadNodeFloat_Response(error_info=stcamera_msgs.msg.ErrorInfo(error_code=0, description=''), value=0.0)
</pre>

* GenICamノード”Gain”(IFloat型)を100に設定します。<br />

``$ ros2 service call /stcamera_launch/dev_142100000000/write_node_float stcamera_msgs/srv/WriteNodeFloat '{genicam_module: "RemoteDevice", genicam_node: "Gain", value: 100}'``<br /> 

<pre>
requester: making request: stcamera_msgs.srv.WriteNodeFloat_Request(genicam_module='RemoteDevice', genicam_node='Gain', value=100.0)

response:
stcamera_msgs.srv.WriteNodeFloat_Response(error_info=stcamera_msgs.msg.ErrorInfo(error_code=0, description=''))
</pre>

### 4.2. フリーランまたはトリガーモードで画像を取得する
stcamera_demosフォルダにあるノード「grabber」はStCameraNodeを使用して、画像データを取得します。C++版とPython版があり、C++版はサービスやトピックの動作確認を兼ねたコードになっており、様々なサービスやトピックに対応しています。

## 5. サポートについて
オムロンセンテック株式会社では本パッケージに関する直接的なサポートは行っておりません。ご質問や不具合等に付きましては、GitHubのisuuesを活用してください。

[GenICam GenApiドキュメント（英語）]:http://www.emva.org/wp-content/uploads/GenICam_Standard_v2_0.pdf
[GenICam GenTLドキュメント（英語）]:http://www.emva.org/wp-content/uploads/GenICam_GenTL_1_5.pdf

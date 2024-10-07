#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> 	//in out pose
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp> 	//in velocity
#include "gmps_msgs/msg/gmps_detect.hpp"                        //in gmps detect
#include "gmps_msgs/msg/gmps_log.hpp"                           //out gmps log
#include "gmps_msgs/msg/rfid.hpp"                               //in rfid

#define RFID_QUEUE_SIZE 10 	//RFIDの検知情報を貯めこむqueueのサイズ //note 10を超えることはまずない
#define SHOW_DEBUG_INFO 0 	//DEBUG_INFOを有効
#define DEBUG_INFO(...) {if (SHOW_DEBUG_INFO) {RCLCPP_INFO(__VA_ARGS__);}}

enum QUEUE_STAMP
{ //可読性のためenumを定義
    NOW = 0, //直近の検知が0
    PREV = 1 //前回の検知が1
};

struct marker_point
{
    uint32_t mm_id; //磁気マーカの通し番号
    uint32_t tag_id;//RFIDタグの番号
    uint8_t mm_kind;//マーカ種別 (not used)
    uint8_t pole;	//磁極 N=1 S=2
    float x;        //埋設位置のX座標
    float y;        //埋設位置のY座標
};

struct marker_info
{
    double x_marker;            //磁気マーカ埋設座標X [m]
    double y_marker;            //磁気マーカ埋設座標Y [m]
    int32_t mm_id;              //磁気マーカ番号 []
    int32_t tag_id_table;       //この磁気マーカに対応して検知されるはずのRFID番号 []
    int32_t tag_id_detected;    //実際に検知したRFID番号
    int32_t mm_kind;            //磁気マーカ種別 []
    int32_t pole;               //磁極 N=1 S=2
    double lateral_deviation;   //磁気センサ横偏差 [m]　+のときGMPSセンサ中心に対し磁気マーカが右側にある
    double delay_dist;          //磁気センサ検出遅延距離 [m]
    double yaw_prev_detected;   //直進判定のためyawを保存
    double mileage;             //走行距離判定のため追加 v2.2.0
    int32_t min_i;              //次回探索の初期値にするための行番号
};

struct rfid_info
{
    int32_t tag_id_detected;
    double mileage;
};


class GMPSLocalizer : public rclcpp::Node
{
private:
    /* Publisher */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gmps_pose_;
    rclcpp::Publisher<gmps_msgs::msg::GmpsLog>::SharedPtr pub_gmps_log_;
    /* Subscriber */
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_prev_pose_;
    rclcpp::Subscription<gmps_msgs::msg::GmpsDetect>::SharedPtr sub_gmps_detect_;
    rclcpp::Subscription<gmps_msgs::msg::Rfid>::SharedPtr sub_rfid_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_velocity_;
    /* Timer */
    rclcpp::TimerBase::SharedPtr timer_;
    /* tf */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;


    /* parameters */
    const bool param_enable_pole_;//
    const bool param_enable_rfid_;
    const double param_tf_x_;                             //baselinkからGMPSセンサ中心までの縦方向オフセット[m] +ならGMPSが前
    const double param_tf_y_;                             //baselinkからGMPSセンサ中心までの横方向オフセット[m] +ならGMPSが左
    const double param_tf_yaw_;                           //baselinkからGMPSセンサまでの角度オフセット[rad] +ならCCW
    const double param_tf_rfid_x_;                        //baselinkからRFIDリーダまでの縦方向オフセット[m]。+ならRFIDリーダが前
    const double param_th_rfid_detect_range_m_;           //RFIDの紐づけ許容誤差[m]
    const double param_th_association_error_dist_m_;      //自己位置から予測したマーカ位置と実際のマーカ敷設位置との許容誤差[m]
    const double param_th_association_break_dist_m_;      //探索を打ち切る閾値[m]。マーカーの最小設置間隔は決まっているので、ある程度近いマーカが見つかったらそれ以上探索する必要はない
    const double param_th_dist_double_marker_m_;          //この距離以内なら二連マーカと判定
    const double param_th_yaw_diff_double_marker_rad_;    //このyaw変化以内なら二連マーカと判定

    const std::string marker_table_csv_name_; //マーカーテーブルのfilename
    const double sigma_x_gmps_;               //X座標の標準偏差
    const double sigma_y_gmps_;               //Y座標の標準偏差
    const double sigma_theta_gmps_;           //yawの標準偏差

    // 強制RFIDマーカ紐付けにて追加
    const double param_marker_d_dist_m_;                  //想定するマーカ間距離
    const double param_th_marker_d_dist_m_;               //マーカ間の誤差。例）マーカ間2.0mで0.2mの場合、2.0±0.2mならマーカ間が正しいと判定
    const double param_th_rfdi_forced_section_change_m_;  //マーカ検知間の距離が本項目を超えたら、強制的にRFIDによるマーカID補正を行う

    geometry_msgs::msg::PoseWithCovarianceStamped prev_pose_;       //自己位置の前回値
    double prev_yaw_;                                               //previous yaw from prev pose
    geometry_msgs::msg::PoseWithCovarianceStamped measurement_pose_;//観測座標
    rclcpp::Time gmps_stamp_;                                       //GMPS検知のタイムスタンプ
    double vx_mps_=0.0;                                             //velocity [m/s]
    double mileage_;                                                //走行距離[m] バックのときは走行距離が減るのでエラー
    rclcpp::Time prev_time_;
    double dt_;

    std::vector<marker_point> marker_table_;//磁気マーカ座標のテーブル
    marker_info marker_queue_[2]={};        //検知したマーカー情報を保存 queueを使うほどでもないので長さ２の配列
    std::queue<rfid_info> rfid_queue_;      //RFIDタグの検知情報のqueue
    bool f_double_marker_;                  //二連マーカとして処理するかどうかのフラグ
    bool f_marker_association_success_;     //RFID or poseによる対応付けに成功したフラグ
    bool f_pose_covariance_high_;           //DRが続いてposeの信頼度が低い場合
    bool f_valid_rfid_detected_;            //RFIDキューから対応するものを見つけたフラグ
    bool f_prev_pose_once_received_=false;  //prev_poseを1回以上受信したフラグ 上位のEKFが初期位置を与えるまで何もpubしない場合のため
    bool f_ignore_pose_;                    //poseの判定をskipするためのフラグ
    double min_rfid_assoc_;                 //RFIDを検知してから磁気マーカ検知するまでの距離　許容最小
    double max_rfid_assoc_;                 //RFIDを検知してから磁気マーカ検知するまでの距離　許容最大

    bool f_rfid_forced_change_req_      = true;     // 強制的にRFIDにより自己位置推定を行う要求フラグ
    bool f_rfid_next_mm_id_change_req_  = false;    // RFIDを受信したので、次のマーカは受信したRFIDをもとに変更する要求フラグ
    double rfid_detection_mileage_;                 // 最新のRFIDを受信した時の距離

    //マーカ予測位置
    float x_predict_marker_;
    float y_predict_marker_;
    float mm_dist_;

    void callbackVelocity(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        vx_mps_ = msg->twist.twist.linear.x;
    }

    //走行距離の更新。粗めの周期で更新しておき、RFIDかGMPSを検知したタイミングで都度更新する。
    void callbackTimer()
    {
        update_mileage();
    }

    void callbackPrevPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        prev_pose_.pose.pose.position.x = msg->pose.pose.position.x;
        prev_pose_.pose.pose.position.y = msg->pose.pose.position.y;
        prev_pose_.pose.pose.position.z = msg->pose.pose.position.z;
        prev_pose_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
        prev_pose_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
        prev_pose_.pose.pose.orientation.z = msg->pose.pose.orientation.z;
        prev_pose_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
        prev_pose_.pose.covariance[6 * 0 + 0] = msg->pose.covariance[6 * 0 + 0]; //初期位置推定完了前かどうかを判別するためにcovarianceを取得する

        //humbleだとtf2::getYaw関数でリンクエラーが発生するので、yawの取得方法を変更
        tf2::Quaternion qua(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(qua).getRPY(roll, pitch, yaw);
        prev_yaw_ = yaw;

        f_prev_pose_once_received_ = true; //一度もprev_poseを受信しなければfalseのまま
    }

    //RFID R/Wからのタグ番号通知を受信したら、RFIDキューをpushする
    void callbackRfid(const gmps_msgs::msg::Rfid::SharedPtr msg)
    {
        DEBUG_INFO(this->get_logger(), "====callback rfid====");
        //check empty and duplicate
        if (rfid_queue_.empty() == false)
        {
            if (rfid_queue_.back().tag_id_detected == msg->tag_id) //同じ番号であれば無視
            {
                DEBUG_INFO(this->get_logger(), "ignore same rfid tag");
                return;
            }
            else if (vx_mps_ < 0.0) //バック中の検知は無視する。停止中はOK
            {
                DEBUG_INFO(this->get_logger(), "ignore this rfid tag. detect RFID tag while reverse driving");
                return;
            }
        }

        //push queue
        update_mileage();
        rfid_info new_info;
        new_info.tag_id_detected = msg->tag_id;
        new_info.mileage = mileage_;
        rfid_queue_.push(new_info);
        DEBUG_INFO(this->get_logger(), "push rfid queue with tag_id=%d at mileage=%.2f", new_info.tag_id_detected, new_info.mileage);

        //check overflow
        if (rfid_queue_.size() > RFID_QUEUE_SIZE)
        {
            DEBUG_INFO(this->get_logger(), "queue size over. erase old rfid queue");
            rfid_queue_.pop();
        }
    }

    //GMPSセンサからの磁気マーカ検知情報を受信したら、
    //磁気マーカqueueをpushし、対応付けと観測座標の計算を行う
    void callbackDetect(const gmps_msgs::msg::GmpsDetect::SharedPtr msg)
    {
        bool ret_bo;

        DEBUG_INFO(this->get_logger(), "====callback detet====");
        gmps_stamp_ = msg->header.stamp;
        update_mileage();
        marker_info blank_info={};
        blank_info.lateral_deviation = msg->lateral_deviation;
        blank_info.pole = msg->pole;
        blank_info.mm_kind = msg->mm_kind;
        blank_info.delay_dist = msg->delay_dist;
        blank_info.yaw_prev_detected = prev_yaw_;
        blank_info.mileage = mileage_;
        blank_info.min_i = marker_queue_[PREV].min_i;
        //push queue
        marker_queue_[PREV] = marker_queue_[NOW];
        marker_queue_[NOW] = blank_info;
        DEBUG_INFO(this->get_logger(), "lateral_deviation=%.3f, pole=%d,delay_dist=%.2f, mileage=%.2f", 
            marker_queue_[NOW].lateral_deviation, marker_queue_[NOW].pole, marker_queue_[NOW].delay_dist, marker_queue_[NOW].mileage);

        //init
        f_marker_association_success_ = false;
        f_valid_rfid_detected_ = false;
        f_double_marker_ = false;
        f_pose_covariance_high_ = (prev_pose_.pose.covariance[6 * 0 + 0] > 1.0) ? true : false;
        f_ignore_pose_ = (f_pose_covariance_high_ == true) || (f_prev_pose_once_received_ == false); //covarianceが高いかposeを一度も受信していない場合
        DEBUG_INFO(this->get_logger(), "prev_x=%.2f, prev_y=%.2f, yaw=%.5f, cov[0]=%.2f",
            prev_pose_.pose.pose.position.x, prev_pose_.pose.pose.position.y,prev_yaw_,prev_pose_.pose.covariance[6 * 0 + 0]);
        DEBUG_INFO(this->get_logger(), "f_ignore_pose = %d", f_ignore_pose_);

        /* rfid_tagと距離による磁気マーカの対応付け。tag_id_detectedを更新 */
        if (param_enable_rfid_==true)
        {
            // マーカ検知間距離が長い？　長い場合は区間が変わった判断して最初のRFID強制マーカ紐付けを行うようにする。
            rfid_forced_too_far_check();

            if (rfid_queue_.empty() == false) // 空でない ?
            {
                DEBUG_INFO(this->get_logger(), "----call association rfid----");
                f_marker_association_success_ = marker_association_by_rfid();
                DEBUG_INFO(this->get_logger(), "----exit association rfid----");

                if ((f_rfid_forced_change_req_ == true ) &&  //  強制的にRFIDにより自己位置推定を行う要求フラグがON？
                    (f_marker_association_success_ == true))  //  RFIDによるmm_id紐付け成功？
                {
                    // 次のマーカの検知時に、強制的にマーカIDを変更フラグON
                    f_rfid_forced_next_mm_id_change_req_on();
                }
            }

            // f_rfid_next_mm_id_change_req_がONの場合は、RFID受信後の次のマーカを強制的にmm_idを変更する
            ret_bo = rfid_forced_next_marker_id_change();
            if (ret_bo == true)	//次のマーカを強制的にmm_idを実施？
            {
                f_marker_association_success_ = true;
            }
        }

        /* RFIDで対応付けができなかった場合はposeによる対応付けをする */
        /* 注）RFIDが無く、covarianceも高い場合は何もできない */
        if ((f_valid_rfid_detected_ == false) && (f_ignore_pose_ == false))
        {
            DEBUG_INFO(this->get_logger(), "----call association pose----");
            /* prev poseによる磁気マーカの対応付け。磁気マーカの埋設位置が特定される。*/
            f_marker_association_success_ = marker_association_by_prev_pose();
            DEBUG_INFO(this->get_logger(), "----exit association pose----");
        }

        publish_log(); //対応付けが失敗した場合でもログはpublishする

        if(f_marker_association_success_ == false)
        {
            RCLCPP_WARN(this->get_logger(), "association fails. this marker was ignored.");
            return; //対応付けに失敗した場合は以降の処理を行わない
        }

        //check if double marker by dist and yaw between PREV and NOW
        //queueが正常に2個あるときのみ二連マーカの判定をする
        if ((marker_queue_[NOW].mm_id > 0) && (marker_queue_[PREV].mm_id > 0)) 
        {
            DEBUG_INFO(this->get_logger(), "----call check double marker----");
            f_double_marker_ = check_double_marker();
            DEBUG_INFO(this->get_logger(), "f_double_marker_ = %d", f_double_marker_);
        }

        /* 観測座標の計算 */
        if (f_double_marker_ == true)
        {

            if (f_rfid_forced_change_req_ == true)
            {
                // RFID２連続受信でき２連マーカと判定したので、強制的にRFIDにより自己位置推定要求をOFFする。
                f_rfid_forced_disable();
            }

            DEBUG_INFO(this->get_logger(), "----call measurement pose by double marker----");
            measurement_double_marker();//二連マーカはX,Y,yaw
            publish_pose();
        }
        else if (f_ignore_pose_ == false) //prev_poseが信頼できないときは単体マーカの処理ができない
        {
            DEBUG_INFO(this->get_logger(), "----call measurement pose by signle marker----");
            measurement_single_marker();//単体マーカはXYのみ
            publish_pose();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "cannot calc measurement pose. this marker was ignored.");
        }
        DEBUG_INFO(this->get_logger(), "====exit callback detet====");
    }


    // マーカ検知間距離が長い？　長い場合は区間が変わった判断して最初のRFID強制マーカ紐付けを行うようにする。
    void  rfid_forced_too_far_check()
    {

        double mileage_between_markers;

        // マーカ検知間距離が長い？　長い場合は区間が変わった判断して最初のRFID強制マーカ紐付けを行うようにする。
        mileage_between_markers = marker_queue_[NOW].mileage -  marker_queue_[PREV].mileage;
        if (mileage_between_markers > param_th_rfdi_forced_section_change_m_)
        {
            f_rfid_forced_change_req_ = true;
            DEBUG_INFO(this->get_logger(), "rfid_forced enables because marker detection interval(%2.2f m) is too far",mileage_between_markers);
        }

    }

    // 次のマーカの検知時に、強制的にマーカIDを変更フラグON
    void f_rfid_forced_next_mm_id_change_req_on()
    {

        f_rfid_next_mm_id_change_req_ = true;		// 次のマーカの検知時に、強制的にマーカIDを変更フラグON
        rfid_detection_mileage_ = mileage_;
    //	DEBUG_INFO(this->get_logger(), "f_rfid_next_mm_id_change_req  on %f",mileage_);

    }

    //  FID強制マーカ紐付けをDisable
    void f_rfid_forced_disable()
    {
        DEBUG_INFO(this->get_logger(), "rfid_forced disables");	

        f_rfid_forced_change_req_ = false;
        f_rfid_next_mm_id_change_req_ = false;		// 次のマーカの検知時に、強制的にマーカIDを変更フラグON
    }

    // RFID受信後の次のマーカを強制的にmm_idを変更する
    bool  rfid_forced_next_marker_id_change()
    {

        int min_i;
        double mileage_diff;
        bool ret_bo=false; // 実施の有無

        mileage_diff = mileage_ - rfid_detection_mileage_;                          //公道なら2m間隔 or 工場内搬送なら50cmm間隔
        if (f_rfid_next_mm_id_change_req_ == true)                                  //次のマーカの検知時に、強制的にマーカIDを変更フラグON?
        {
            if (abs(mileage_diff-param_marker_d_dist_m_) < param_th_marker_d_dist_m_) // マーカ検知間の距離誤差がparam_th_marker_d_dist_m_以内？
            {

                DEBUG_INFO(this->get_logger(), "rfid_forced  marker_queue[NOW]_mm_id %d  marker_queue[PREV]_mm_id  %d",marker_queue_[NOW].mm_id,marker_queue_[PREV].mm_id);


                min_i = marker_queue_[PREV].min_i+1;
                marker_queue_[NOW].min_i = min_i;
                marker_queue_[NOW].x_marker = marker_table_[min_i].x;
                marker_queue_[NOW].y_marker = marker_table_[min_i].y;
                marker_queue_[NOW].mm_id = marker_table_[min_i].mm_id;
                marker_queue_[NOW].tag_id_table = marker_table_[min_i].tag_id;

                ret_bo  = true;

                DEBUG_INFO(this->get_logger(), "rfid_forced assocation marker_queue[NOW]_mm_id %d",marker_queue_[NOW].mm_id);
                f_rfid_forced_disable();
            } else
            {
                DEBUG_INFO(this->get_logger(), "rfid_forced assocation NG  dist between marker is %2.2f ",mileage_diff);
            }

        }

        return ret_bo;

    }

    //RFIDキューから今検知したマーカーに対応するものを探してtag_id_detectedに保存する
    bool marker_association_by_rfid()
    {
        /* associate RFID queue */
        DEBUG_INFO(this->get_logger(), "associate RFID queue");
        f_valid_rfid_detected_ = false; //RFIDキューから対応するものを見つけたフラグ
        bool f_tag_id_matched = false; //検知したタグ番号に一致するマーカテーブルを見つけた

        //RFIDを検知してから磁気マーカを検知するまでの走行距離の最大最小。磁気マーカの距離遅延を考慮する。
        min_rfid_assoc_ = param_tf_rfid_x_ - (param_tf_x_ - marker_queue_[NOW].delay_dist) - param_th_rfid_detect_range_m_;
        max_rfid_assoc_ = param_tf_rfid_x_ - (param_tf_x_ - marker_queue_[NOW].delay_dist) + param_th_rfid_detect_range_m_;
        max_rfid_assoc_ = std::max(0.0, max_rfid_assoc_); //オフセットの設定が悪いとmaxが負になることもありうるため、saturationを入れる

        const double mileage_gmps = marker_queue_[NOW].mileage;
        DEBUG_INFO(this->get_logger(), "RFID queue size=%ld", rfid_queue_.size());
        /* RFIDのqueueを先頭（最も古い）から距離をチェックしていく */
        while (rfid_queue_.empty() == false)
        {
            const double mileage_rfid = rfid_queue_.front().mileage;
            const double mileage_rfid_gmps = mileage_gmps - mileage_rfid; //RFIDを検知した後にGMPSを検知するはずなので、これは常に正であるべき

            if (mileage_rfid_gmps < min_rfid_assoc_) //最も古いRFIDの走行距離がMIN未満であればこれ以上探索する意味はない
            {
                DEBUG_INFO(this->get_logger(), "dist < min  ---->  QUIT tag_id=%d", rfid_queue_.front().tag_id_detected);
                break;
            }
            else if (mileage_rfid_gmps > max_rfid_assoc_) //走行距離がMAXを超えている場合は破棄する。
            {
                DEBUG_INFO(this->get_logger(), "dist > max  ---->  CONTINUE tag_id=%d", rfid_queue_.front().tag_id_detected);
                rfid_queue_.pop();
                continue;
            }
            else //min <= dist <= max
            {
                DEBUG_INFO(this->get_logger(), "min < dist < max  ---->  POP!!!");
                marker_queue_[NOW].tag_id_detected = rfid_queue_.front().tag_id_detected;

#if 0  //　240410 次に同じTAG IDを受信すると、再度rfid_queueに登録されるので、本処理を削除
                rfid_queue_.pop();
#endif
                f_valid_rfid_detected_ = true;
                DEBUG_INFO(this->get_logger(), "tag_id_detected=%d corresponding to this marker", marker_queue_[NOW].tag_id_detected)
                break;
            }
        }

        /* 適切なqueueが無ければreturn. 以降の処理を行わない */
        if (f_valid_rfid_detected_ == false)
        {
            DEBUG_INFO(this->get_logger(), "cannot find valid distance RFID queue. continue pose association");
            return false;
        }

        /* associate marker by rfid*/
        const int i_start = marker_queue_[PREV].min_i; //前回のマーカから探索を開始する
        const int i_end = i_start + marker_table_.size(); //設定としては常に全点探索。十分近いものが見つかったら中断する

        DEBUG_INFO(this->get_logger(), "i_start=%d, i_end=%d",i_start, i_end);
        for (int i=i_start; i < i_end; i++)
        {
            int j=i; //indexは本来0~size()-1であるべきところ、ループ対応で範囲を超える場合があるので中間変数を使う //debug %でよい
            if (j < 0) //始点を超えたらループ
            {
                j = j + marker_table_.size();
            }
            else if (static_cast<size_t>(j) >= marker_table_.size()) //終点を超えたらループ
            {
                j = j - marker_table_.size();
            }
            //DEBUG_INFO(this->get_logger(), "j=%d", j);

            /* RFID による探索 */
            if(static_cast<uint32_t>(marker_queue_[NOW].tag_id_detected) == marker_table_[j].tag_id) //検知したタグ番号と同じ番号が見つかるまで進める
            {
                RCLCPP_INFO(this->get_logger(), "TAG_ID %d matched at table index %d", marker_queue_[NOW].tag_id_detected, j);
                marker_queue_[NOW].min_i = j;
                f_tag_id_matched = true; //note RFIDの対応付けに成功したら、以降の処理で失敗してもprev poseの探索を行わない
                break;
            }
            else
            {
                //DEBUG_INFO(this->get_logger(), "CONTINUE by RFID");
                continue;
            }
        }

        /* マーカーテーブルに一致するタグ番号が見つからなければ終了。以降の処理を行わない */
        if (f_tag_id_matched == false)
        {
            RCLCPP_WARN(this->get_logger(), "cannot find TAG_ID %d in marker table.", marker_queue_[NOW].tag_id_detected);
            return false;
        }

        /* 極性による照合 */
        if ((param_enable_pole_ == true) &&
            (marker_queue_[NOW].pole != marker_table_[marker_queue_[NOW].min_i].pole))
        {
            RCLCPP_WARN(this->get_logger(), "invalid pole. gmps detected=%d, table expected=%d", marker_queue_[NOW].pole, marker_table_[marker_queue_[NOW].min_i].pole);
            return false;
        }

        /* 距離による照合 */
        //prev_poseが信頼できる場合は距離による照合を行う
        if (f_ignore_pose_ == false)
        {
            const double predict_marker_x = prev_pose_.pose.pose.position.x
                + (param_tf_x_) * cos(prev_yaw_) - (param_tf_y_) * sin(prev_yaw_) //from base_link to gpms_center
                + (- marker_queue_[NOW].delay_dist) * cos(prev_yaw_ + param_tf_yaw_) + (+ marker_queue_[NOW].lateral_deviation) * sin(prev_yaw_ + param_tf_yaw_); //from gmps_center to marker
            const double predict_marker_y = prev_pose_.pose.pose.position.y
                + (param_tf_x_) * sin(prev_yaw_) + (param_tf_y_) * cos(prev_yaw_) //from base_link to gmps_center
                + (- marker_queue_[NOW].delay_dist) * sin(prev_yaw_ + param_tf_yaw_) - (+ marker_queue_[NOW].lateral_deviation) * cos(prev_yaw_ + param_tf_yaw_); //from gmps_center to marker

            const double dist = calc_dist(marker_table_[marker_queue_[NOW].min_i].x - predict_marker_x,
                marker_table_[marker_queue_[NOW].min_i].y - predict_marker_y);

            x_predict_marker_ = predict_marker_x;
            y_predict_marker_ = predict_marker_y;
            mm_dist_ = dist;
            if (dist > param_th_association_error_dist_m_)
            {
                marker_queue_[NOW].mm_id = -2; //後解析で分かるようにエラー番号を入れておく
                RCLCPP_ERROR(this->get_logger(),
                "Ignore detect. dist=%.2f > threshold %.2f", dist, param_th_association_error_dist_m_);
                RCLCPP_ERROR(this->get_logger(),
                "predicted marker pos x=%.2f,y=%.2f is too far from table defined pos x=%.2f,y=%.2f",
                    predict_marker_x, predict_marker_y, marker_table_[marker_queue_[NOW].min_i].x, marker_table_[marker_queue_[NOW].min_i].y);

                return false;
            }
        }

        /* 正常なreturn */
        DEBUG_INFO(this->get_logger(), "return associate rfid");
        marker_queue_[NOW].x_marker = marker_table_[marker_queue_[NOW].min_i].x;
        marker_queue_[NOW].y_marker = marker_table_[marker_queue_[NOW].min_i].y;
        marker_queue_[NOW].mm_id = marker_table_[marker_queue_[NOW].min_i].mm_id;
        marker_queue_[NOW].tag_id_table = marker_table_[marker_queue_[NOW].min_i].tag_id;
        RCLCPP_INFO(this->get_logger(),
            "push marker queue with mm_id=%d, tag_id=%d, X=%.2lf, Y=%.2lf",
            marker_queue_[NOW].mm_id, marker_queue_[NOW].tag_id_table,
            marker_queue_[NOW].x_marker, marker_queue_[NOW].y_marker);
        return true;
    }

    //自己位置からテーブルを探索する
    bool marker_association_by_prev_pose()
    {
        DEBUG_INFO(this->get_logger(), "association pose called");
        //baselinkからsensor, markerまでの座標変換
        geometry_msgs::msg::Point predict_marker_pose; //自己位置から予想したマーカー位置
        predict_marker_pose.x = prev_pose_.pose.pose.position.x
            + (param_tf_x_) * cos(prev_yaw_) - (param_tf_y_) * sin(prev_yaw_) //from base_link to gmps_center
            + (- marker_queue_[NOW].delay_dist) * cos(prev_yaw_ + param_tf_yaw_) + (+ marker_queue_[NOW].lateral_deviation) * sin(prev_yaw_ + param_tf_yaw_); //from gmps_center to marker
        predict_marker_pose.y = prev_pose_.pose.pose.position.y
            + (param_tf_x_) * sin(prev_yaw_) + (param_tf_y_) * cos(prev_yaw_) //from base_link to gmps_center
            + (- marker_queue_[NOW].delay_dist) * sin(prev_yaw_ + param_tf_yaw_) - (+ marker_queue_[NOW].lateral_deviation) * cos(prev_yaw_ + param_tf_yaw_); //from gmps_center to marker

        //association
        double min_dist = 100000;
        int min_i = 100000;
        double dist;

        const int i_start = marker_queue_[PREV].min_i;
        const int i_end = i_start + marker_table_.size(); //設定としては常に全点探索。十分近いものが見つかったら中断する
        DEBUG_INFO(this->get_logger(), "i_start=%d, i_end=%d",i_start, i_end);
        for (int i=i_start; i < i_end; i++)
        {
            int j=i; //indexは本来0~size()-1であるべきところ、ループ対応で範囲を超える場合があるので中間変数を使う //debug %でよい
            if (j < 0) //始点を超えたらループ
            {
                j = j + marker_table_.size();
            }
            else if (static_cast<uint32_t>(j) >= marker_table_.size()) //終点を超えたらループ
            {
                j = j - marker_table_.size();
            }
            //DEBUG_INFO(this->get_logger(), "j=%d", j);

            /* 極性による探索 */
            if ((param_enable_pole_ == true) &&
                (marker_queue_[NOW].pole != marker_table_[j].pole)) //極性が一致しないときはスキップ
            {
                //DEBUG_INFO(this->get_logger(), "CONTINUE by POLE");
                continue;
            }

            dist = calc_dist(marker_table_[j].x - predict_marker_pose.x, marker_table_[j].y - predict_marker_pose.y);
            // マーカーテーブルの全点探索
            if (dist < min_dist)
            {
                min_dist = dist;
                min_i = j;

                // かなり近いマーカがあればそこで終了
                if(min_dist < param_th_association_break_dist_m_)
                {
                    RCLCPP_INFO(this->get_logger(), "terminate list search by %.2f",param_th_association_break_dist_m_);
                    break;
                }
            }
        }

        x_predict_marker_ = predict_marker_pose.x;
        y_predict_marker_ = predict_marker_pose.y;
        mm_dist_ = min_dist;
        DEBUG_INFO(this->get_logger(), "check association error dist");
        //　エラー判定　閾値以内のマーカが見つかっていないなら以降の処理は行わない
        if (min_dist > param_th_association_error_dist_m_)
        {
            marker_queue_[NOW].mm_id = -1; //後解析で分かるようにエラー番号を入れておく
                RCLCPP_ERROR(this->get_logger(),
                "Ignore detect. min_dist=%.2f > threshold %.2f", min_dist, param_th_association_error_dist_m_);
                RCLCPP_ERROR(this->get_logger(),
                "predicted marker pos x=%.2f,y=%.2f is too far from table defined pos x=%.2f,y=%.2f",
                    predict_marker_pose.x, predict_marker_pose.y, marker_table_[min_i].x, marker_table_[min_i].y);


            return false;
        }

        DEBUG_INFO(this->get_logger(), "return associate pose");
        marker_queue_[NOW].min_i = min_i;
        marker_queue_[NOW].x_marker = marker_table_[min_i].x;
        marker_queue_[NOW].y_marker = marker_table_[min_i].y;
        marker_queue_[NOW].mm_id = marker_table_[min_i].mm_id;
        marker_queue_[NOW].tag_id_table = marker_table_[min_i].tag_id;
        RCLCPP_INFO(this->get_logger(),
            "push marker queue with mm_id=%d, tag_id=%d, X=%.2lf, Y=%.2lf",
            marker_queue_[NOW].mm_id, marker_queue_[NOW].tag_id_table,
            marker_queue_[NOW].x_marker, marker_queue_[NOW].y_marker);

        return true;
    }

    // 二連マーカとして処理するかどうかの判定
    bool check_double_marker()
    {
        //一個目のマーカから二個目のマーカを検知するまでの走行距離が閾値以内であること
        //note 走行距離の差分ではなく、マーカ敷設座標の距離にすべきか
        const double mileage_between_markers = marker_queue_[NOW].mileage - marker_queue_[PREV].mileage;

        //一個目のマーカと二個目のマーカでyawが大きく変化していないこと
        //note twist.angular.zを使えばyawrateの積分から判定できるはず
        const double yaw_diff_between_markers =
            marker_queue_[NOW].yaw_prev_detected - marker_queue_[PREV].yaw_prev_detected;

        // 同じmm_id?
        if (marker_queue_[NOW].mm_id  == marker_queue_[PREV].mm_id)
        {
            //異常扱い

            RCLCPP_ERROR(this->get_logger(),
            "check_double_marker mm_id is same id=%d,%d", marker_queue_[NOW].mm_id ,marker_queue_[PREV].mm_id);

            //marker_queue_[NOW].mm_id = -3;    //debug publish_log()の発行タイミングを変更する必要があり、今回は保留..

            return false;
        }

        return ((0.1 < mileage_between_markers) && //あまりにも近距離で連続して検知した場合は誤検知なので無視する
                (mileage_between_markers < param_th_dist_double_marker_m_) &&
                (abs(yaw_diff_between_markers) < param_th_yaw_diff_double_marker_rad_));
    }

    void measurement_single_marker()
    {
        //markerからsensor, baselinkまでの変換
        measurement_pose_.pose.pose.position.x = marker_queue_[NOW].x_marker
            - (- marker_queue_[NOW].delay_dist) * cos(prev_yaw_ + param_tf_yaw_) - (+ marker_queue_[NOW].lateral_deviation) * sin(prev_yaw_ + param_tf_yaw_) //from marker to gmps_center
            - (param_tf_x_) * cos(prev_yaw_) + (param_tf_y_) * sin(prev_yaw_); //from gmps_center to base_link

        measurement_pose_.pose.pose.position.y = marker_queue_[NOW].y_marker
            - (- marker_queue_[NOW].delay_dist) * sin(prev_yaw_ + param_tf_yaw_) + (+ marker_queue_[NOW].lateral_deviation) * cos(prev_yaw_ + param_tf_yaw_) //from marker to gmps_center
            - (param_tf_x_) * sin(prev_yaw_) - (param_tf_y_) * cos(prev_yaw_); //from gmps_center to base_link

        //z, qはprev_poseから継承
        measurement_pose_.pose.pose.position.z = prev_pose_.pose.pose.position.z;
        measurement_pose_.pose.pose.orientation = prev_pose_.pose.pose.orientation;
        //covariance
        measurement_pose_.pose.covariance[6*0+0] = sigma_x_gmps_ * sigma_x_gmps_; //x*x
        measurement_pose_.pose.covariance[6*1+1] = sigma_y_gmps_ * sigma_y_gmps_; //y*y
        measurement_pose_.pose.covariance[6*5+5] = sigma_theta_gmps_ * sigma_theta_gmps_; //yaw*yaw
    }

    void measurement_double_marker()
    {
        //磁気マーカ間のyawを計算 1個目[PREV]を基準とした2個目[NOW]へのyaw
        const double dx = marker_queue_[NOW].x_marker - marker_queue_[PREV].x_marker;
        const double dy = marker_queue_[NOW].y_marker - marker_queue_[PREV].y_marker;
        const double yaw_marker = atan2(dy, dx);
        //二つの横変位からdthetaを計算
        //マーカ間距離には測量値の差分を使用する
        const double dist_marker = calc_dist(dx,dy);
        const double dtheta = asin((marker_queue_[NOW].lateral_deviation - marker_queue_[PREV].lateral_deviation) / dist_marker );
        //その合計が現在のyaw
        const double yaw_gmps = yaw_marker + dtheta;

        DEBUG_INFO(this->get_logger(), "PREV x=%.2f,y=%.2f,latdev=%.3f, NOW x=%.2f,y=%.2f,side=%.3f",
            marker_queue_[PREV].x_marker, marker_queue_[PREV].y_marker, marker_queue_[PREV].lateral_deviation,
            marker_queue_[NOW].x_marker, marker_queue_[NOW].y_marker, marker_queue_[NOW].lateral_deviation);
        DEBUG_INFO(this->get_logger(),"yaw_marker=%.5f, dtheta=%.5f, yaw_gmps=%.5f", yaw_marker, dtheta, yaw_gmps);

        //markerからsensor, baselinkまでの変換
        measurement_pose_.pose.pose.position.x = marker_queue_[NOW].x_marker
            - (- marker_queue_[NOW].delay_dist) * cos(yaw_gmps + param_tf_yaw_) - (+ marker_queue_[NOW].lateral_deviation) * sin(yaw_gmps + param_tf_yaw_) //from marker to gmps_center
            - (param_tf_x_) * cos(yaw_gmps) + (param_tf_y_) * sin(yaw_gmps); //from gmps_center to base_link
        measurement_pose_.pose.pose.position.y = marker_queue_[NOW].y_marker
            - (- marker_queue_[NOW].delay_dist) * sin(yaw_gmps + param_tf_yaw_) + (+ marker_queue_[NOW].lateral_deviation) * cos(yaw_gmps + param_tf_yaw_) //from marker to gmps_center
            - (param_tf_x_) * sin(yaw_gmps) - (param_tf_y_) * cos(yaw_gmps); //from gmps_center to base_link
        //zはprev_poseから継承
        measurement_pose_.pose.pose.position.z = prev_pose_.pose.pose.position.z;
        //quaternion
        tf2::Quaternion qua;
        qua.setRPY(0,0,yaw_gmps);
        measurement_pose_.pose.pose.orientation.x = qua.x();
        measurement_pose_.pose.pose.orientation.y = qua.y();
        measurement_pose_.pose.pose.orientation.z = qua.z();
        measurement_pose_.pose.pose.orientation.w = qua.w();
        //covariance
        measurement_pose_.pose.covariance[6*0+0] = sigma_x_gmps_ * sigma_x_gmps_; //x*x
        measurement_pose_.pose.covariance[6*1+1] = sigma_y_gmps_ * sigma_y_gmps_; //y*y
        measurement_pose_.pose.covariance[6*5+5] = sigma_theta_gmps_ * sigma_theta_gmps_; //yaw*yaw
    }

    //解析用にマーカ番号と横変位が紐づいたデータをpublishする
    void publish_log()
    {
        DEBUG_INFO(this->get_logger(), "publish log");
        gmps_msgs::msg::GmpsLog log_msg;
        log_msg.header.stamp = gmps_stamp_;
        log_msg.header.frame_id = "gmps";
        //マーカ検知
        log_msg.lateral_deviation = marker_queue_[NOW].lateral_deviation;
        log_msg.pole = marker_queue_[NOW].pole;
        log_msg.mm_kind = marker_queue_[NOW].mm_kind;
        //RFID検知
        log_msg.tag_id_detected = marker_queue_[NOW].tag_id_detected;
        //対応付けが成功した場合のみ
        log_msg.mm_id = marker_queue_[NOW].mm_id;
        log_msg.tag_id_table = marker_queue_[NOW].tag_id_table;
        log_msg.x_marker = marker_queue_[NOW].x_marker;
        log_msg.y_marker = marker_queue_[NOW].y_marker;
        //マーカ予測位置と最近傍マーカとの距離をログに残すために追加
        log_msg.x_predict_marker = x_predict_marker_;
        log_msg.y_predict_marker = y_predict_marker_;
        log_msg.mm_dist = mm_dist_;
        pub_gmps_log_->publish(log_msg);
    }

    void publish_pose()
    {
        DEBUG_INFO(this->get_logger(), "publish /gmps_pose with X=%.2f, Y=%.2f",
            measurement_pose_.pose.pose.position.x, measurement_pose_.pose.pose.position.y);
        //poseのpublish
        geometry_msgs::msg::PoseWithCovarianceStamped gmps_pose; //output measurement pose for publish
        gmps_pose.header.frame_id = "map"; //frameIDはmap
        gmps_pose.header.stamp = gmps_stamp_; //time stamp は磁気マーカを検知したタイミングなので、detect msgから継承
        gmps_pose.pose.pose.position.x = measurement_pose_.pose.pose.position.x; //X, Yはcallbackで計算
        gmps_pose.pose.pose.position.y = measurement_pose_.pose.pose.position.y;
        gmps_pose.pose.pose.position.z = measurement_pose_.pose.pose.position.z; //Zはprev poseから継承
        gmps_pose.pose.pose.orientation = measurement_pose_.pose.pose.orientation; //qはprev pose または 二連マーカ
        gmps_pose.pose.covariance[6 * 0 + 0] = measurement_pose_.pose.covariance[6*0+0]; //x*x
        gmps_pose.pose.covariance[6 * 1 + 1] = measurement_pose_.pose.covariance[6*1+1]; //y*y
        gmps_pose.pose.covariance[6 * 5 + 5] = measurement_pose_.pose.covariance[6*5+5]; //yaw*yaw

        pub_gmps_pose_->publish(gmps_pose);

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "gmps_link";
        transformStamped.transform.translation.x = measurement_pose_.pose.pose.position.x;
        transformStamped.transform.translation.y = measurement_pose_.pose.pose.position.y;
        transformStamped.transform.translation.z = measurement_pose_.pose.pose.position.z;

        transformStamped.transform.rotation.x = measurement_pose_.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = measurement_pose_.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = measurement_pose_.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = measurement_pose_.pose.pose.orientation.w;

        tf_br_->sendTransform(transformStamped);
    }

    //磁気マーカテーブルcsvの取り込み
    void load_marker_csv(const char *csvname)
    {
        //fp
        if (NULL == csvname)
        {
            RCLCPP_ERROR(this->get_logger(),"csvname is empty");
            exit(1);
        }
        char buf[256];

        FILE *fp = fopen(csvname, "r");
        if (fp == NULL)
        {
            RCLCPP_ERROR(this->get_logger(),"could not open marker_table file %s. please confirm filepath is correct.", csvname);
            exit(1);
        }

        int32_t ret = 0;
        ret = fscanf(fp, "%s\n", buf); // headerは捨てる

        int32_t num[4];
        double data[2];
        marker_point record; //vector代入用の一時変数
        while((ret = fscanf(fp, "%d, %d, %d, %d, %lf, %lf\n",
        &num[0], &num[1], &num[2], &num[3], &data[0], &data[1] )) != EOF)
            {
                record.mm_id = num[0];
                record.tag_id = num[1];
                record.mm_kind = num[2];
                record.pole = num[3];
                record.x = data[0];
                record.y = data[1];
                marker_table_.push_back(record);
                if(record.mm_id == 0)
                {
                    RCLCPP_WARN(this->get_logger(), "marker table include mm_id=0. this may cause unintended consequence" );
                }
            }
        fclose(fp);
    }

    //距離計算
    double calc_dist(const double dx,const double dy)
    {
        return(sqrt(dx*dx + dy*dy));
    }

    //速度情報をもとに走行距離を更新する
    void update_mileage()
    {
        dt_ = (get_clock()->now() - prev_time_).seconds();
        mileage_ = mileage_ + vx_mps_ * dt_;
        prev_time_ = get_clock()->now();
    }


public:
    GMPSLocalizer(const rclcpp::NodeOptions &node_options)
        : rclcpp::Node("gmps_localizer", node_options)
        /* parameters */
        , param_enable_pole_(declare_parameter<bool>("enable_pole", true))
        , param_enable_rfid_(declare_parameter<bool>("enable_rfid", false))
        , param_tf_x_(declare_parameter<double>("tf_x", 1.0))
        , param_tf_y_(declare_parameter<double>("tf_y", 0.0))
        , param_tf_yaw_(declare_parameter<double>("tf_yaw", 0.0))
        , param_tf_rfid_x_(declare_parameter<double>("tf_rfid_x", 2.0))
        , param_th_rfid_detect_range_m_(declare_parameter<double>("th_rfid_detect_range_m", 0.5))
        , param_th_association_error_dist_m_(declare_parameter<double>("th_association_error_dist_m", 0.5))
        , param_th_association_break_dist_m_(declare_parameter<double>("th_association_break_dist_m", 0.3))
        , param_th_dist_double_marker_m_(declare_parameter<double>("th_dist_double_marker_m", 2.5))
        , param_th_yaw_diff_double_marker_rad_(declare_parameter<double>("th_yaw_diff_double_marker_rad", 0.05))
        , marker_table_csv_name_(declare_parameter<std::string>("marker_table_csv_name", "gmps_driver/DATA/marker.csv"))
        , sigma_x_gmps_(declare_parameter<double>("sigma_x_gmps", 0.07))
        , sigma_y_gmps_(declare_parameter<double>("sigma_y_gmps", 0.07))
        , sigma_theta_gmps_(declare_parameter<double>("sigma_theta_gmps", 0.1))
        // 強制RFIDマーカ紐付けにて追加
        , param_marker_d_dist_m_(declare_parameter<double>("marker_d_dist_m", 2.0))
        , param_th_marker_d_dist_m_(declare_parameter<double>("th_marker_d_dist_m",0.2))
        , param_th_rfdi_forced_section_change_m_(declare_parameter<double>("th_rfdi_forced_section_change_m",100.0))
    {
        RCLCPP_INFO(this->get_logger(), "constructor start");

        /* Publisher */
        pub_gmps_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("out_gmps_pose", rclcpp::QoS(1));
        pub_gmps_log_ = this->create_publisher<gmps_msgs::msg::GmpsLog>("out_gmps_log", rclcpp::QoS(1));

        /* Subscriber */
        sub_prev_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "in_prev_pose", rclcpp::QoS(1), std::bind(&GMPSLocalizer::callbackPrevPose, this, std::placeholders::_1));
        sub_gmps_detect_ = this->create_subscription<gmps_msgs::msg::GmpsDetect>(
            "in_gmps_detect", rclcpp::SensorDataQoS(), std::bind(&GMPSLocalizer::callbackDetect, this, std::placeholders::_1));
        sub_rfid_ = this->create_subscription<gmps_msgs::msg::Rfid>(
            "in_rfid", rclcpp::SensorDataQoS(), std::bind(&GMPSLocalizer::callbackRfid, this, std::placeholders::_1));
        sub_velocity_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "in_velocity", rclcpp::QoS(1), std::bind(&GMPSLocalizer::callbackVelocity, this, std::placeholders::_1));
        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
            std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

        /* Timer */
        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Rate(50).period(), std::bind(&GMPSLocalizer::callbackTimer, this));
        prev_time_ = get_clock()->now();

        //marker.csvのロード
        RCLCPP_INFO(this->get_logger(), "load marker csv");
        load_marker_csv( (char*)marker_table_csv_name_.c_str() );

        RCLCPP_INFO(this->get_logger(), "constructor end");
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    std::shared_ptr<GMPSLocalizer> node;
    try{//メンバ初期化リスト(コンストラクタ宣言と中括弧の間の記述)で例外がthrowされた場合のtry-catch
        node = std::make_shared<GMPSLocalizer>(node_options);
    } catch(std::runtime_error &e) {//ros2の例外型の継承元クラスはstd::runtime_error
		RCLCPP_ERROR_STREAM(rclcpp::get_logger("gmps_localizer"), e.what());
		return -1;
	}

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

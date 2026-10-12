#ifndef PROTOCAL_H
#define PROTOCAL_H
#include "utils.h"
#include <cstdint>
#include <QtGlobal>

NAMESPACE_BEGIN{ namespace server{
    /**
     * @brief 远程控制协议的协议端口号
     */
    const quint16 PORT_CMD_TCP = 2356;

    /**
     * @brief 远程传输协议的数据传输端口号
     */
    const quint16 PORT_DATA_UDP = 2355;

    /**
     * @brief 远程控制协议的魔数，对应于`UWSC`
     */
    const uint32_t MAGIC_TCP = 0x55575343;

    /**
     * @brief 远程传输协议的数据传输端口号的魔数，对应于`UWSD`
     */
    const uint32_t MAGIC_UDP = 0x55575344;

    /**
     * @brief 协议版本号，当前为0x0001
     */
    const uint16_t PROTO_VERSION = 0x0001;

    /**
     * @brief 远程传输协议的数据订阅类型
     */
    enum UdpSubscriptionType {
        SubNone         = 0,        // 不订阅任何数据
        SubStatus       = 1 << 0,   // 订阅状态数据
        SubImage        = 1 << 1,   // 订阅图像数据
        SubPointCloud   = 1 << 2,   // 订阅点云数据
    };

    /**
     * @brief 远程传输协议的UDP数据包类型
     */
    enum UdpDataType{
        UdpTypeStatus       = 1,    // 状态数据
        UdpTypeImage        = 2,    // 图像数据
        UdpTypePointCloud   = 3,    // 点云数据
    };

    #pragma pack(push, 1)  // 1字节对齐，确保跨平台/语言兼容

    /**
     * @brief TCP 协议帧头，确保1字节对齐，共占用16字节
     */
    struct TcpFrameHeader {
        uint32_t magic;        // 魔数（用于快速过滤无关请求），暂定为0x55575343 (UWScan的ASCII码)
        uint16_t version;      // 协议版本: 0x0001
        uint16_t error_code;   // 错误码，请求为0，响应成功为1，失败情况会给对应的错误码
        uint32_t req_id;       // 请求号，用于响应时定位
        uint32_t data_length;  // 数据长度
    };

    /**
     * @brief UDP 协议帧头，确保1字节对齐，共占用20字节。
     *
     * transfer_id、fragment_index 和 fragment_count 用于在应用层
     * 重组超过 MTU 的图片和点云数据。
     */
    struct UdpFrameHeader {
        uint32_t magic;        // 0x55575344
        uint16_t version;      // 0x0001
        uint16_t data_type;    // 1-Status, 2-Image, 3-Cloud
        uint32_t timestamp;    // 时间戳
        uint32_t transfer_id;  // 同一份数据的传输 ID
        uint16_t fragment_index; // 分片序号，从0开始
        uint16_t fragment_count; // 分片总数
    };

    #pragma pack(pop)  // 恢复默认对齐
}}


#endif // PROTOCAL_H

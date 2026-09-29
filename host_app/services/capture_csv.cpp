/** CSV 明确标记演示来源；导出完整记录，不用绘图抽点代替原始数据。 */
#include "capture_csv.h"
#include <QFile>
#include <cmath>
#include <stdexcept>

namespace yg::host
{
namespace
{
QByteArray header()
{
    return "#YG_MC_HOST_DEMO_CAPTURE_V1\ndemo_node,time_model_s,position_rad,speed_rad_s,iq_a,"
           "position_"
           "target_rad,speed_reference_rad_s,iq_reference_a,voltage_v,temperature_c,gap_before\n";
}
QString number(double value)
{
    return QString::number(value, 'g', 17);
}
QString optional_number(std::optional<double> value)
{
    return value ? number(*value) : QString();
}
double parse(const QString &text)
{
    bool valid = false;
    const auto value = text.toDouble(&valid);
    if (!valid || !std::isfinite(value))
    {
        throw std::runtime_error("CSV 含无效数值");
    }
    return value;
}
std::optional<double> parse_optional(const QString &text)
{
    return text.isEmpty() ? std::nullopt : std::optional<double>(parse(text));
}
} // namespace

void Capture_Write(const QString &path, const std::vector<ScopeSample> &samples)
{
    if (samples.empty() || samples.size() > 30000)
    {
        throw std::runtime_error("记录为空或超过容量");
    }
    QByteArray bytes = header();
    for (const auto &s : samples)
    {
        const QStringList fields{QString::number(s.demo_node),
                                 number(s.time_s),
                                 number(s.position_rad),
                                 number(s.speed_rad_s),
                                 number(s.iq_a),
                                 optional_number(s.position_reference_rad),
                                 optional_number(s.speed_reference_rad_s),
                                 optional_number(s.iq_reference_a),
                                 number(s.voltage_v),
                                 number(s.temperature_c),
                                 s.gap_before ? "1" : "0"};
        bytes += fields.join(',').toUtf8() + '\n';
    }
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::NewOnly) || file.write(bytes) != bytes.size())
    {
        throw std::runtime_error("CSV 导出失败，请选择新的输出文件");
    }
}

std::vector<ScopeSample> Capture_Read(const QString &path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() > 16 * 1024 * 1024)
    {
        throw std::runtime_error("CSV 不存在或超过 16 MiB");
    }
    const auto bytes = file.readAll();
    if (!bytes.startsWith(header()))
    {
        throw std::runtime_error("CSV 不是受支持的演示记录");
    }
    std::vector<ScopeSample> samples;
    for (const auto &line : QString::fromUtf8(bytes.mid(header().size())).split('\n'))
    {
        if (line.isEmpty())
        {
            continue;
        }
        const auto f = line.split(',');
        if (f.size() != 11 || samples.size() >= 30000 || (f[10] != "0" && f[10] != "1"))
        {
            throw std::runtime_error("CSV 字段或容量错误");
        }
        ScopeSample s;
        bool node_valid = false;
        const auto node = f[0].toUInt(&node_valid);
        if (!node_valid || node == 0 || node >= 255)
        {
            throw std::runtime_error("CSV 演示节点错误");
        }
        s.demo_node = quint8(node);
        if (!samples.empty() && s.demo_node != samples.front().demo_node)
        {
            throw std::runtime_error("CSV 混合了不同演示节点");
        }
        s.time_s = parse(f[1]);
        s.position_rad = parse(f[2]);
        s.speed_rad_s = parse(f[3]);
        s.iq_a = parse(f[4]);
        s.position_reference_rad = parse_optional(f[5]);
        s.speed_reference_rad_s = parse_optional(f[6]);
        s.iq_reference_a = parse_optional(f[7]);
        s.voltage_v = parse(f[8]);
        s.temperature_c = parse(f[9]);
        s.gap_before = f[10] == "1";
        if (s.time_s < 0 || (!samples.empty() && s.time_s <= samples.back().time_s))
        {
            throw std::runtime_error("CSV 时间倒退或重复");
        }
        samples.push_back(s);
    }
    if (samples.empty())
    {
        throw std::runtime_error("CSV 没有样本");
    }
    return samples;
}
} // namespace yg::host

#pragma once
#include "services/workbench_session.h"
#include <QWidget>

namespace yg::host
{
/** 复用原工作台三组同步绘图；仅消费服务样本，冻结不停止服务记录。 */
class ScopeWidget final : public QWidget
{
  public:
    /**
     * @brief 创建 SI 单位的三组同步图。
     * @param parent Qt 父控件，可为空。
     */
    explicit ScopeWidget(QWidget *parent = nullptr);
    /**
     * @brief 替换显示样本，冻结时保持当前画面。
     * @param samples 有界样本，控件保存显示副本。
     */
    void Scope_SetSamples(const std::deque<ScopeSample> &samples);
    /**
     * @brief 冻结或恢复显示，不改变采集状态。
     * @param frozen 是否冻结。
     */
    void Scope_SetFrozen(bool frozen);
    /**
     * @brief 修改三个分图共用的时间窗口。
     * @param seconds 窗口秒数，限制 1..60。
     */
    void Scope_SetWindow(double seconds);
    /**
     * @brief 定位当前记录中的显示时间。
     * @param fraction 0..1，空值表示实时末尾。
     */
    void Scope_SetReplayFraction(std::optional<double> fraction);

  protected:
    void paintEvent(QPaintEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void leaveEvent(QEvent *) override;

  private:
    std::deque<ScopeSample> samples_;
    bool frozen_ = false;
    double seconds_ = 10, cursor_ = -1;
    std::optional<double> replay_fraction_;
};
} // namespace yg::host

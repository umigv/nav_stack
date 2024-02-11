namespace nav2_gradient_costmap_plugin
{

class GradientLayer : public nav2_costmap_2d::Layer
{
    GradientLayer::onInitialize()
    {
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node_->get_parameter(name_ + "." + "enabled", enabled_);

        need_recalculation_ = false;
    }

    GradientLayer::updateBounds()
    {
        if (need_recalculation_ == true){
            //re-calculate windown bounds
        }
        //update window bounds
    }


}
}
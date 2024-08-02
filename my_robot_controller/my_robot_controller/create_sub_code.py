# Source: 
# https://github.com/ros2/rclpy
# rclpy/rclpy/rclpy/node.py
# Line 1579-1655

def create_subscription(
        self,
        msg_type: Type[MsgT],
        topic: str,
        callback: Callable[[MsgT], None],
        qos_profile: Union[QoSProfile, int],
        *,
        callback_group: Optional[CallbackGroup] = None,
        event_callbacks: Optional[SubscriptionEventCallbacks] = None,
        qos_overriding_options: Optional[QoSOverridingOptions] = None,
        raw: bool = False
    ) -> Subscription[MsgT]:
        """
        Create a new subscription.

        :param msg_type: The type of ROS messages the subscription will subscribe to.
        :param topic: The name of the topic the subscription will subscribe to.
        :param callback: A user-defined callback function that is called when a message is
            received by the subscription.
        :param qos_profile: A QoSProfile or a history depth to apply to the subscription.
            In the case that a history depth is provided, the QoS history is set to
            KEEP_LAST, the QoS history depth is set to the value
            of the parameter, and all other QoS settings are set to their default values.
        :param callback_group: The callback group for the subscription. If ``None``, then the
            default callback group for the node is used.
        :param event_callbacks: User-defined callbacks for middleware events.
        :param raw: If ``True``, then received messages will be stored in raw binary
            representation.
        """
        qos_profile = self._validate_qos_or_depth_parameter(qos_profile)

        callback_group = callback_group or self.default_callback_group

        try:
            final_topic = self.resolve_topic_name(topic)
        except RuntimeError:
            # if it's name validation error, raise a more appropriate exception.
            try:
                self._validate_topic_or_service_name(topic)
            except InvalidTopicNameException as ex:
                raise ex from None
            # else reraise the previous exception
            raise

        if qos_overriding_options is None:
            qos_overriding_options = QoSOverridingOptions([])
        _declare_qos_parameters(
            Subscription, self, final_topic, qos_profile, qos_overriding_options)

        # this line imports the typesupport for the message module if not already done
        failed = False
        check_is_valid_msg_type(msg_type)
        try:
            with self.handle:
                subscription_object = _rclpy.Subscription(
                    self.handle, msg_type, topic, qos_profile.get_c_qos_profile())
        except ValueError:
            failed = True
        if failed:
            self._validate_topic_or_service_name(topic)

        try:
            subscription = Subscription(
                subscription_object, msg_type,
                topic, callback, callback_group, qos_profile, raw,
                event_callbacks=event_callbacks or SubscriptionEventCallbacks())
        except Exception:
            subscription_object.destroy_when_not_in_use()
            raise
        callback_group.add_entity(subscription)
        self._subscriptions.append(subscription)
        self._wake_executor()

        for event_handler in subscription.event_handlers:
            self.add_waitable(event_handler)

        return subscription

from enum import Enum
from functools import wraps
import struct


XSensHeader = struct.Struct("!6sIBBIBBBBhh")


class XSensPacketType(Enum):
    """ Packet types """

    PacketEulerPose = "01"
    PacketQuaternionPose = "02"
    PacketCartesianPose = "03"
    PacketUnityPose = "05"
    PacketCharacterMetaInformation = "12"
    PacketCharacterScaling = "13"
    PacketJointAngleData = "20"
    PacketLinearSegmentKinematics = "21"
    PacketAngularSegmentKinematics = "22"
    PacketMotionTrackerKinematics = "23"
    PacketCenterOfMass = "24"
    PacketTimecode = "25"


class XSensEulerQuaternionSegmentType(Enum):
    Pelvis = 0
    L5 = 1
    L3 = 2
    T12 = 3
    T8 = 4
    Neck = 5
    Head = 6
    Right_Shoulder = 7
    Right_Upper_Arm = 8
    Right_Forearm = 9
    Right_Hand = 10
    Left_Shoulder = 11
    Left_Upper_Arm = 12
    Left_Forearm = 13
    Left_Hand = 14
    Right_Upper_Leg = 15
    Right_Lower_Leg = 16
    Right_Foot = 17
    Right_Toe = 18
    Left_Upper_Leg = 19
    Left_Lower_Leg = 20
    Left_Foot = 21
    Left_Toe = 22
    Prop1 = 24
    Prop2 = 25
    Prop3 = 26
    Prop4 = 27


class XSensEulerUnity3DSegmentType(Enum):
    Pelvis = 0
    Right_Upper_Leg = 1
    Right_Lower_Leg = 2
    Right_Foot = 3
    Right_Toe = 4
    Left_Upper_Leg = 5
    Left_Lower_Leg = 6
    Left_Foot = 7
    Left_Toe = 8
    L5 = 9
    L3 = 10
    T12 = 11
    T8 = 12
    Left_Shoulder = 13
    Left_Upper_Arm = 14
    Left_Forearm = 15
    Left_Hand = 16
    Right_Shoulder = 17
    Right_Upper_Arm = 18
    Right_Forearm = 19
    Right_Hand = 20
    Neck = 21
    Head = 22
    Prop1 = 24
    Prop2 = 25
    Prop3 = 26
    Prop4 = 27


class PacketProcessor(object):
    """ Helper decorator for extracting correct packet data based on type """

    def __init__(self, component_enum, base_component):
        self.component_enum = component_enum
        self.base_component = base_component

    def __call__(self, function):
        @wraps(function)
        def wrapper(*args, **kwargs):
            calling_object = args[0]
            component_position = calling_object.components.get(
                self.component_enum, None
            )
            if component_position is None:
                return None

            component_position, component_info = XSensPacket._get_exact(
                self.base_component, calling_object.data, component_position
            )

            return (
                component_info,
                function(
                    *args,
                    component_info=component_info,
                    data=calling_object.data,
                    component_position=component_position,
                    **kwargs
                ),
            )

        return wrapper


# noinspection PyTypeChecker
class XSensPacket(object):
    """Packet containing data measured and streamed with XSens MVN.

    Too check for existence a specific packet:

        from xsenslsl.packet import XSensPacketType
        if XSensPacketType.Component3d in packet.components:
            header, markers = packet.get_3d_markers()

    Component retriever functions will return None if a component is not in the packet.

    """

    def __init__(self, data):
        self.data = data

        self.timestamp, self.framenumber, component_count = RTDataQRTPacket.unpack_from(
            data, 0
        )

        self.components = {}
        position = RTDataQRTPacket.size
        for _ in range(component_count):
            c_size, c_type = RTComponentData.unpack_from(data, position)
            self.components[QRTComponentType(c_type)] = position + RTComponentData.size
            position += c_size

    @staticmethod
    def _get_exact(component_type, data, position):
        value = component_type._make(component_type.format.unpack_from(data, position))
        position += component_type.format.size
        return position, value

    @staticmethod
    def _get_tuple(component_type, data, position):
        value = component_type._make(
            [component_type.format.unpack_from(data, position)]
        )
        position += component_type.format.size
        return position, value

    # @staticmethod
    # def _get_2d_markers(data, component_info, component_position, index=None):
    #     components = []
    #     append_components = components.append
    #     for camera in range(component_info.camera_count):
    #         component_position, camera_info = QRTPacket._get_exact(
    #             RT2DCamera, data, component_position
    #         )

    #         if index is None or index == camera:
    #             marker_list = []
    #             append_marker = marker_list.append
    #             append_components(marker_list)

    #             for _ in range(camera_info.marker_count):
    #                 component_position, marker = QRTPacket._get_exact(
    #                     RT2DMarker, data, component_position
    #                 )
    #                 append_marker(marker)
    #         else:
    #             component_position += RT2DMarker.format.size * camera_info.marker_count

    #     return components

    # @staticmethod
    # def _get_3d_markers(type_, component_info, data, component_position):
    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.marker_count):
    #         component_position, position = QRTPacket._get_exact(
    #             type_, data, component_position
    #         )
    #         append_components(position)

    #     return components

    # @PacketProcessor(QRTComponentType.ComponentTimecode, RTTimeComponent)
    # def get_timecode(self, component_info=None, data=None, component_position=None):
    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.timecode_count):
    #         time_component, timecode = QRTPacket._get_exact(
    #             RTTime, data, component_position
    #         )
    #         append_components(timecode)

    #     return components

    # @PacketProcessor(QRTComponentType.ComponentAnalog, RTAnalogComponent)
    # def get_analog(self, component_info=None, data=None, component_position=None):
    #     """Get analog data."""
    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.device_count):
    #         component_position, device = QRTPacket._get_exact(
    #             RTAnalogDevice, data, component_position
    #         )
    #         if device.sample_count > 0:
    #             component_position, sample_number = QRTPacket._get_exact(
    #                 RTSampleNumber, data, component_position
    #             )

    #             RTAnalogChannel.format = struct.Struct(
    #                 RTAnalogChannel.format_str % device.sample_count
    #             )
    #             for _ in range(device.channel_count):
    #                 component_position, channel = QRTPacket._get_tuple(
    #                     RTAnalogChannel, data, component_position
    #                 )
    #                 append_components((device, sample_number, channel))

    #     return components

    # @PacketProcessor(QRTComponentType.ComponentAnalogSingle, RTAnalogComponent)
    # def get_analog_single(
    #     self, component_info=None, data=None, component_position=None
    # ):
    #     """Get a single analog data channel."""
    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.device_count):
    #         component_position, device = QRTPacket._get_exact(
    #             RTAnalogDeviceSingle, data, component_position
    #         )

    #         RTAnalogDeviceSamples.format = struct.Struct(
    #             RTAnalogDeviceSamples.format_str % device.channel_count
    #         )
    #         component_position, sample = QRTPacket._get_tuple(
    #             RTAnalogDeviceSamples, data, component_position
    #         )
    #         append_components((device, sample))
    #     return components

    # @PacketProcessor(QRTComponentType.ComponentForce, RTForceComponent)
    # def get_force(self, component_info=None, data=None, component_position=None):
    #     """Get force data."""
    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.plate_count):
    #         component_position, plate = QRTPacket._get_exact(
    #             RTForcePlate, data, component_position
    #         )
    #         force_list = []
    #         for _ in range(plate.force_count):
    #             component_position, force = QRTPacket._get_exact(
    #                 RTForce, data, component_position
    #             )
    #             force_list.append(force)
    #         append_components((plate, force_list))
    #     return components

    # @PacketProcessor(QRTComponentType.ComponentForceSingle, RTForceComponent)
    # def get_force_single(self, component_info=None, data=None, component_position=None):
    #     """Get a single force data channel."""
    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.plate_count):
    #         component_position, plate = QRTPacket._get_exact(
    #             RTForcePlateSingle, data, component_position
    #         )
    #         component_position, force = QRTPacket._get_exact(
    #             RTForce, data, component_position
    #         )
    #         append_components((plate, force))
    #     return components

    # @PacketProcessor(QRTComponentType.Component6d, RT6DComponent)
    # def get_6d(self, component_info=None, data=None, component_position=None):
    #     """Get 6D data."""
    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.body_count):
    #         component_position, position = QRTPacket._get_exact(
    #             RT6DBodyPosition, data, component_position
    #         )
    #         component_position, matrix = QRTPacket._get_tuple(
    #             RT6DBodyRotation, data, component_position
    #         )
    #         append_components((position, matrix))
    #     return components

    # @PacketProcessor(QRTComponentType.Component6dRes, RT6DComponent)
    # def get_6d_residual(self, component_info=None, data=None, component_position=None):
    #     """Get 6D data with residual."""
    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.body_count):
    #         component_position, position = QRTPacket._get_exact(
    #             RT6DBodyPosition, data, component_position
    #         )
    #         component_position, matrix = QRTPacket._get_tuple(
    #             RT6DBodyRotation, data, component_position
    #         )
    #         component_position, residual = QRTPacket._get_exact(
    #             RT6DBodyResidual, data, component_position
    #         )
    #         append_components((position, matrix, residual))
    #     return components

    # @PacketProcessor(QRTComponentType.Component6dEuler, RT6DComponent)
    # def get_6d_euler(self, component_info=None, data=None, component_position=None):
    #     """Get 6D data with euler rotations."""
    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.body_count):
    #         component_position, position = QRTPacket._get_exact(
    #             RT6DBodyPosition, data, component_position
    #         )
    #         component_position, euler = QRTPacket._get_exact(
    #             RT6DBodyEuler, data, component_position
    #         )
    #         append_components((position, euler))
    #     return components

    # @PacketProcessor(QRTComponentType.Component6dEulerRes, RT6DComponent)
    # def get_6d_euler_residual(
    #     self, component_info=None, data=None, component_position=None
    # ):
    #     """Get 6D data with residuals and euler rotations."""
    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.body_count):
    #         component_position, position = QRTPacket._get_exact(
    #             RT6DBodyPosition, data, component_position
    #         )
    #         component_position, euler = QRTPacket._get_exact(
    #             RT6DBodyEuler, data, component_position
    #         )
    #         component_position, residual = QRTPacket._get_exact(
    #             RT6DBodyResidual, data, component_position
    #         )
    #         append_components((position, euler, residual))
    #     return components

    # @PacketProcessor(QRTComponentType.ComponentImage, RTImageComponent)
    # def get_image(self, component_info=None, data=None, component_position=None):
    #     """Get image."""
    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.image_count):
    #         component_position, image_info = QRTPacket._get_exact(
    #             RTImage, data, component_position
    #         )
    #         append_components(
    #             (
    #                 image_info,
    #                 data[
    #                     component_position : component_position + image_info.image_size
    #                 ],
    #             )
    #         )
    #         component_position += image_info.image_size
    #     return components

    # @PacketProcessor(QRTComponentType.Component3d, RT3DComponent)
    # def get_3d_markers(self, component_info=None, data=None, component_position=None):
    #     """Get 3D markers."""
    #     return self._get_3d_markers(
    #         RT3DMarkerPosition, component_info, data, component_position
    #     )

    # @PacketProcessor(QRTComponentType.Component3dRes, RT3DComponent)
    # def get_3d_markers_residual(
    #     self, component_info=None, data=None, component_position=None
    # ):
    #     """Get 3D markers with residual."""
    #     return self._get_3d_markers(
    #         RT3DMarkerPositionResidual, component_info, data, component_position
    #     )

    # @PacketProcessor(QRTComponentType.Component3dNoLabels, RT3DComponent)
    # def get_3d_markers_no_label(
    #     self, component_info=None, data=None, component_position=None
    # ):
    #     """Get 3D markers without label."""
    #     return self._get_3d_markers(
    #         RT3DMarkerPositionNoLabel, component_info, data, component_position
    #     )

    # @PacketProcessor(QRTComponentType.Component3dNoLabelsRes, RT3DComponent)
    # def get_3d_markers_no_label_residual(
    #     self, component_info=None, data=None, component_position=None
    # ):
    #     """Get 3D markers without label with residual."""
    #     return self._get_3d_markers(
    #         RT3DMarkerPositionNoLabelResidual, component_info, data, component_position
    #     )

    # @PacketProcessor(QRTComponentType.Component2d, RT2DComponent)
    # def get_2d_markers(
    #     self, component_info=None, data=None, component_position=None, index=None
    # ):
    #     """Get 2D markers.

    #     :param index: Specify which camera to get 2D from, will be returned as
    #                   first entry in the returned array.
    #     """
    #     return self._get_2d_markers(
    #         data, component_info, component_position, index=index
    #     )

    # @PacketProcessor(QRTComponentType.Component2dLin, RT2DComponent)
    # def get_2d_markers_linearized(
    #     self, component_info=None, data=None, component_position=None, index=None
    # ):
    #     """Get 2D linearized markers.

    #     :param index: Specify which camera to get 2D from, will be returned as
    #                   first entry in the returned array.
    #     """

    #     return self._get_2d_markers(
    #         data, component_info, component_position, index=index
    #     )

    # @PacketProcessor(QRTComponentType.ComponentSkeleton, RTSkeletonComponent)
    # def get_skeletons(self, component_info=None, data=None, component_position=None):
    #     """Get skeletons
    #     """

    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.skeleton_count):
    #         component_position, info = QRTPacket._get_exact(
    #             RTSegmentCount, data, component_position
    #         )

    #         segments = []
    #         for __ in range(info.segment_count):
    #             component_position, segment = QRTPacket._get_exact(
    #                 RTSegmentId, data, component_position
    #             )
    #             component_position, position = QRTPacket._get_exact(
    #                 RTSegmentPosition, data, component_position
    #             )
    #             component_position, rotation = QRTPacket._get_exact(
    #                 RTSegmentRotation, data, component_position
    #             )

    #             segments.append((segment.id, position, rotation))
    #         append_components(segments)
    #     return components

    # @PacketProcessor(QRTComponentType.ComponentGazeVector, RTGazeVectorComponent)
    # def get_gaze_vectors(self, component_info=None, data=None, component_position=None):
    #     """Get gaze vectors
    #     """

    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.vector_count):
    #         component_position, info = QRTPacket._get_exact(
    #             RTGazeVectorInfo, data, component_position
    #         )

    #         samples = []
    #         if info.sample_count > 0:
    #             for _ in range(info.sample_count):
    #                 component_position, unit_vector = QRTPacket._get_exact(
    #                     RTGazeVectorUnitVector, data, component_position
    #                 )

    #                 component_position, position = QRTPacket._get_exact(
    #                     RTGazeVectorPosition, data, component_position
    #                 )

    #                 samples.append((unit_vector, position))

    #         append_components((info, samples))

    #     return components

    # @PacketProcessor(QRTComponentType.ComponentEyeTracker, RTEyeTrackerComponent)
    # def get_eye_trackers(self, component_info=None, data=None, component_position=None):
    #     """Get eye trackers
    #     """

    #     components = []
    #     append_components = components.append
    #     for _ in range(component_info.eye_tracker_count):
    #         component_position, info = QRTPacket._get_exact(
    #             RTEyeTrackerInfo, data, component_position
    #         )

    #         samples = []
    #         if info.sample_count > 0:
    #             for _ in range(info.sample_count):
    #                 component_position, diameter = QRTPacket._get_exact(
    #                     RTEyeTrackerDiameter, data, component_position
    #                 )
    #                 samples.append(diameter)

    #         append_components((info, samples))

    #     return components

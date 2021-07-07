// <auto-generated>
//     Generated by the protocol buffer compiler.  DO NOT EDIT!
//     source: parameter_server.proto
// </auto-generated>
#pragma warning disable 0414, 1591
#region Designer generated code

using grpc = global::Grpc.Core;

namespace Parameterserver {
  public static partial class ParameterServer
  {
    static readonly string __ServiceName = "parameterserver.ParameterServer";

    static void __Helper_SerializeMessage(global::Google.Protobuf.IMessage message, grpc::SerializationContext context)
    {
      #if !GRPC_DISABLE_PROTOBUF_BUFFER_SERIALIZATION
      if (message is global::Google.Protobuf.IBufferMessage)
      {
        context.SetPayloadLength(message.CalculateSize());
        global::Google.Protobuf.MessageExtensions.WriteTo(message, context.GetBufferWriter());
        context.Complete();
        return;
      }
      #endif
      context.Complete(global::Google.Protobuf.MessageExtensions.ToByteArray(message));
    }

    static class __Helper_MessageCache<T>
    {
      public static readonly bool IsBufferMessage = global::System.Reflection.IntrospectionExtensions.GetTypeInfo(typeof(global::Google.Protobuf.IBufferMessage)).IsAssignableFrom(typeof(T));
    }

    static T __Helper_DeserializeMessage<T>(grpc::DeserializationContext context, global::Google.Protobuf.MessageParser<T> parser) where T : global::Google.Protobuf.IMessage<T>
    {
      #if !GRPC_DISABLE_PROTOBUF_BUFFER_SERIALIZATION
      if (__Helper_MessageCache<T>.IsBufferMessage)
      {
        return parser.ParseFrom(context.PayloadAsReadOnlySequence());
      }
      #endif
      return parser.ParseFrom(context.PayloadAsNewBuffer());
    }

    static readonly grpc::Marshaller<global::Parameterserver.GetParamRequest> __Marshaller_parameterserver_GetParamRequest = grpc::Marshallers.Create(__Helper_SerializeMessage, context => __Helper_DeserializeMessage(context, global::Parameterserver.GetParamRequest.Parser));
    static readonly grpc::Marshaller<global::Parameterserver.ParamValue> __Marshaller_parameterserver_ParamValue = grpc::Marshallers.Create(__Helper_SerializeMessage, context => __Helper_DeserializeMessage(context, global::Parameterserver.ParamValue.Parser));
    static readonly grpc::Marshaller<global::Parameterserver.SetParamRequest> __Marshaller_parameterserver_SetParamRequest = grpc::Marshallers.Create(__Helper_SerializeMessage, context => __Helper_DeserializeMessage(context, global::Parameterserver.SetParamRequest.Parser));
    static readonly grpc::Marshaller<global::Common.Empty> __Marshaller_common_Empty = grpc::Marshallers.Create(__Helper_SerializeMessage, context => __Helper_DeserializeMessage(context, global::Common.Empty.Parser));

    static readonly grpc::Method<global::Parameterserver.GetParamRequest, global::Parameterserver.ParamValue> __Method_GetParameter = new grpc::Method<global::Parameterserver.GetParamRequest, global::Parameterserver.ParamValue>(
        grpc::MethodType.Unary,
        __ServiceName,
        "GetParameter",
        __Marshaller_parameterserver_GetParamRequest,
        __Marshaller_parameterserver_ParamValue);

    static readonly grpc::Method<global::Parameterserver.SetParamRequest, global::Common.Empty> __Method_SetParameter = new grpc::Method<global::Parameterserver.SetParamRequest, global::Common.Empty>(
        grpc::MethodType.Unary,
        __ServiceName,
        "SetParameter",
        __Marshaller_parameterserver_SetParamRequest,
        __Marshaller_common_Empty);

    /// <summary>Service descriptor</summary>
    public static global::Google.Protobuf.Reflection.ServiceDescriptor Descriptor
    {
      get { return global::Parameterserver.ParameterServerReflection.Descriptor.Services[0]; }
    }

    /// <summary>Base class for server-side implementations of ParameterServer</summary>
    [grpc::BindServiceMethod(typeof(ParameterServer), "BindService")]
    public abstract partial class ParameterServerBase
    {
      public virtual global::System.Threading.Tasks.Task<global::Parameterserver.ParamValue> GetParameter(global::Parameterserver.GetParamRequest request, grpc::ServerCallContext context)
      {
        throw new grpc::RpcException(new grpc::Status(grpc::StatusCode.Unimplemented, ""));
      }

      public virtual global::System.Threading.Tasks.Task<global::Common.Empty> SetParameter(global::Parameterserver.SetParamRequest request, grpc::ServerCallContext context)
      {
        throw new grpc::RpcException(new grpc::Status(grpc::StatusCode.Unimplemented, ""));
      }

    }

    /// <summary>Client for ParameterServer</summary>
    public partial class ParameterServerClient : grpc::ClientBase<ParameterServerClient>
    {
      /// <summary>Creates a new client for ParameterServer</summary>
      /// <param name="channel">The channel to use to make remote calls.</param>
      public ParameterServerClient(grpc::ChannelBase channel) : base(channel)
      {
      }
      /// <summary>Creates a new client for ParameterServer that uses a custom <c>CallInvoker</c>.</summary>
      /// <param name="callInvoker">The callInvoker to use to make remote calls.</param>
      public ParameterServerClient(grpc::CallInvoker callInvoker) : base(callInvoker)
      {
      }
      /// <summary>Protected parameterless constructor to allow creation of test doubles.</summary>
      protected ParameterServerClient() : base()
      {
      }
      /// <summary>Protected constructor to allow creation of configured clients.</summary>
      /// <param name="configuration">The client configuration.</param>
      protected ParameterServerClient(ClientBaseConfiguration configuration) : base(configuration)
      {
      }

      public virtual global::Parameterserver.ParamValue GetParameter(global::Parameterserver.GetParamRequest request, grpc::Metadata headers = null, global::System.DateTime? deadline = null, global::System.Threading.CancellationToken cancellationToken = default(global::System.Threading.CancellationToken))
      {
        return GetParameter(request, new grpc::CallOptions(headers, deadline, cancellationToken));
      }
      public virtual global::Parameterserver.ParamValue GetParameter(global::Parameterserver.GetParamRequest request, grpc::CallOptions options)
      {
        return CallInvoker.BlockingUnaryCall(__Method_GetParameter, null, options, request);
      }
      public virtual grpc::AsyncUnaryCall<global::Parameterserver.ParamValue> GetParameterAsync(global::Parameterserver.GetParamRequest request, grpc::Metadata headers = null, global::System.DateTime? deadline = null, global::System.Threading.CancellationToken cancellationToken = default(global::System.Threading.CancellationToken))
      {
        return GetParameterAsync(request, new grpc::CallOptions(headers, deadline, cancellationToken));
      }
      public virtual grpc::AsyncUnaryCall<global::Parameterserver.ParamValue> GetParameterAsync(global::Parameterserver.GetParamRequest request, grpc::CallOptions options)
      {
        return CallInvoker.AsyncUnaryCall(__Method_GetParameter, null, options, request);
      }
      public virtual global::Common.Empty SetParameter(global::Parameterserver.SetParamRequest request, grpc::Metadata headers = null, global::System.DateTime? deadline = null, global::System.Threading.CancellationToken cancellationToken = default(global::System.Threading.CancellationToken))
      {
        return SetParameter(request, new grpc::CallOptions(headers, deadline, cancellationToken));
      }
      public virtual global::Common.Empty SetParameter(global::Parameterserver.SetParamRequest request, grpc::CallOptions options)
      {
        return CallInvoker.BlockingUnaryCall(__Method_SetParameter, null, options, request);
      }
      public virtual grpc::AsyncUnaryCall<global::Common.Empty> SetParameterAsync(global::Parameterserver.SetParamRequest request, grpc::Metadata headers = null, global::System.DateTime? deadline = null, global::System.Threading.CancellationToken cancellationToken = default(global::System.Threading.CancellationToken))
      {
        return SetParameterAsync(request, new grpc::CallOptions(headers, deadline, cancellationToken));
      }
      public virtual grpc::AsyncUnaryCall<global::Common.Empty> SetParameterAsync(global::Parameterserver.SetParamRequest request, grpc::CallOptions options)
      {
        return CallInvoker.AsyncUnaryCall(__Method_SetParameter, null, options, request);
      }
      /// <summary>Creates a new instance of client from given <c>ClientBaseConfiguration</c>.</summary>
      protected override ParameterServerClient NewInstance(ClientBaseConfiguration configuration)
      {
        return new ParameterServerClient(configuration);
      }
    }

    /// <summary>Creates service definition that can be registered with a server</summary>
    /// <param name="serviceImpl">An object implementing the server-side handling logic.</param>
    public static grpc::ServerServiceDefinition BindService(ParameterServerBase serviceImpl)
    {
      return grpc::ServerServiceDefinition.CreateBuilder()
          .AddMethod(__Method_GetParameter, serviceImpl.GetParameter)
          .AddMethod(__Method_SetParameter, serviceImpl.SetParameter).Build();
    }

    /// <summary>Register service method with a service binder with or without implementation. Useful when customizing the  service binding logic.
    /// Note: this method is part of an experimental API that can change or be removed without any prior notice.</summary>
    /// <param name="serviceBinder">Service methods will be bound by calling <c>AddMethod</c> on this object.</param>
    /// <param name="serviceImpl">An object implementing the server-side handling logic.</param>
    public static void BindService(grpc::ServiceBinderBase serviceBinder, ParameterServerBase serviceImpl)
    {
      serviceBinder.AddMethod(__Method_GetParameter, serviceImpl == null ? null : new grpc::UnaryServerMethod<global::Parameterserver.GetParamRequest, global::Parameterserver.ParamValue>(serviceImpl.GetParameter));
      serviceBinder.AddMethod(__Method_SetParameter, serviceImpl == null ? null : new grpc::UnaryServerMethod<global::Parameterserver.SetParamRequest, global::Common.Empty>(serviceImpl.SetParameter));
    }

  }
}
#endregion

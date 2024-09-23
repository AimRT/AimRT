// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstdint>

namespace aimrt::plugins::grpc_plugin::http2 {
// The following source code is from trpc-cpp.
// Copied and modified from
// https://github.com/trpc-group/trpc-cpp/blob/v1.2.0/trpc/util/http/status.h

/// HTTP status codes as registered with IANA.
/// @sa : https://www.iana.org/assignments/http-status-codes/http-status-codes.xhtml
///
/// HTTP status defined in RFC-7231
/// @sa: https://www.rfc-editor.org/rfc/rfc7231#section-6
enum class HttpStatus : uint32_t {
  /// @brief 1xx: Informational - Request received, continuing process.
  kContinue = 100,            // RFC 7231, 6.2.1
  kSwitchingProtocols = 101,  // RFC 7231, 6.2.2
  kProcessing = 102,          // RFC 2518, 10.1
  kEarlyHints = 103,          // RFC 8297

  /// @brief 2xx: Success - The action was successfully received, understood, and accepted.
  kOk = 200,                    // RFC 7231, 6.3.1
  kCreated = 201,               // RFC 7231, 6.3.2
  kAccepted = 202,              // RFC 7231, 6.3.3
  kNonAuthoritativeInfo = 203,  // RFC 7231, 6.3.4
  kNoContent = 204,             // RFC 7231, 6.3.5
  kResetContent = 205,          // RFC 7231, 6.3.6
  kPartialContent = 206,        // RFC 7233, 4.1
  kMultiStatus = 207,           // RFC 4918, 11.1
  kAlreadyReported = 208,       // RFC 5842, 7.1
  kIAmUsed = 226,               // RFC 3229, 10.4.1

  /// @brief 3xx: Redirection - Further action must be taken in order to complete the request.
  kMultipleChoices = 300,    // RFC 7231, 6.4.1
  kMovedPermanently = 301,   // RFC 7231, 6.4.2
  kFound = 302,              // RFC 7231, 6.4.3
  kSeeOther = 303,           // RFC 7231, 6.4.4
  kNotModified = 304,        // RFC 7232, 4.1
  kUseProxy = 305,           // RFC 7231, 6.4.5
  kUnused306 = 306,          // RFC 7231, 6.4.6 (Unused)
  kTemporaryRedirect = 307,  // RFC 7231, 6.4.7
  kPermanentRedirect = 308,  // RFC 7538, 3

  /// @brief 4xx: Client Error - The request contains bad syntax or cannot be fulfilled.
  kBadRequest = 400,                    // RFC 7231, 6.5.1
  kUnauthorized = 401,                  // RFC 7235, 3.1
  kPaymentRequired = 402,               // RFC 7231, 6.5.2
  kForbidden = 403,                     // RFC 7231, 6.5.3
  kNotFound = 404,                      // RFC 7231, 6.5.4
  kMethodNotAllowed = 405,              // RFC 7231, 6.5.5
  kNotAcceptable = 406,                 // RFC 7231, 6.5.6
  kProxyAuthRequired = 407,             // RFC 7235, 3.2
  kRequestTimeout = 408,                // RFC 7231, 6.5.7
  kConflict = 409,                      // RFC 7231, 6.5.8
  kGone = 410,                          // RFC 7231, 6.5.9
  kLengthRequired = 411,                // RFC 7231, 6.5.10
  kPreconditionFailed = 412,            // RFC 7232, 4.2
  kRequestEntityTooLarge = 413,         // RFC 7231, 6.5.11
  kRequestUriTooLong = 414,             // RFC 7231, 6.5.12
  kUnsupportedMediaType = 415,          // RFC 7231, 6.5.13
  kRequestedRangeNotSatisfiable = 416,  // RFC 7233, 4.4
  kExpectationFailed = 417,             // RFC 7231, 6.5.14
  kIAmATeapot = 418,                    // RFC 7168, 2.3.3
  kMisdirectedRequest = 421,            // RFC 7540, 9.1.2
  kUnprocessableEntity = 422,           // RFC 4918, 11.2
  kLocked = 423,                        // RFC 4918, 11.3
  kFailedDependency = 424,              // RFC 4918, 11.4
  kTooEarly = 425,                      // RFC 8470, 5.2.
  kUpgradeRequired = 426,               // RFC 7231, 6.5.15
  kPreconditionRequired = 428,          // RFC 6585, 3
  kTooManyRequests = 429,               // RFC 6585, 4
  kRequestHeaderFieldsTooLarge = 431,   // RFC 6585, 5
  kUnavailableForLegalReasons = 451,    // RFC 7725, 3
  kClientClosedRequest = 499,           // Extended: client has closed connection.

  /// @brief 5xx: Server Error - The server failed to fulfill an apparently valid request.
  kInternalServerError = 500,            // RFC 7231, 6.6.1
  kNotImplemented = 501,                 // RFC 7231, 6.6.2
  kBadGateway = 502,                     // RFC 7231, 6.6.3
  kServiceUnavailable = 503,             // RFC 7231, 6.6.4
  kGatewayTimeout = 504,                 // RFC 7231, 6.6.5
  kHttpVersionNotSupported = 505,        // RFC 7231, 6.6.6
  kVariantAlsoNegotiates = 506,          // RFC 2295, 8.1
  kInsufficientStorage = 507,            // RFC 4918, 11.5
  kLoopDetected = 508,                   // RFC 5842, 7.2
  kNotExtended = 510,                    // RFC 2774, 7
  kNetworkAuthenticationRequired = 511,  // RFC 6585,
};
// End of source code from trpc-cpp.

}  // namespace aimrt::plugins::grpc_plugin::http2
"""
Positional Encoding Module
支持两种条件输入编码方式:
1. Feature Engineering: 手工设计的特征变换
2. Positional Encoding: 基于正弦/余弦的位置编码
"""

import numpy as np


class PositionalEncoder:
    """
    位置编码器,使用正弦和余弦函数对条件输入进行编码
    参考 Transformer 中的 Positional Encoding
    """
    def __init__(self, encoding_dim=5, max_value=10.0):
        """
        Args:
            encoding_dim: 编码输出的维度(必须为奇数或偶数)
            max_value: 输入值的预期最大值,用于归一化
        """
        self.encoding_dim = encoding_dim
        self.max_value = max_value
        
        # 生成不同频率的位置编码
        # 使用类似 Transformer 的频率设置
        self.frequencies = self._generate_frequencies()
    
    def _generate_frequencies(self):
        """生成不同频率的波形"""
        # 频率从高到低,覆盖不同尺度的变化
        freqs = []
        for i in range(self.encoding_dim // 2):
            # 频率呈指数递减
            freq = 1.0 / (10000 ** (2 * i / self.encoding_dim))
            freqs.append(freq)
        return np.array(freqs)
    
    def encode(self, value):
        """
        对单个标量值进行位置编码
        
        Args:
            value: 标量输入值(如楼梯高度)
            
        Returns:
            encoding: shape (encoding_dim,) 的编码向量
        """
        # 归一化输入值
        normalized_value = value / self.max_value
        
        # 生成编码
        encoding = []
        
        # 添加原始归一化值
        encoding.append(normalized_value)
        
        # 添加正弦/余弦编码
        for freq in self.frequencies:
            encoding.append(np.sin(2 * np.pi * freq * normalized_value))
            encoding.append(np.cos(2 * np.pi * freq * normalized_value))
            
            # 如果已经达到目标维度,停止
            if len(encoding) >= self.encoding_dim:
                break
        
        # 确保输出维度正确
        encoding = np.array(encoding[:self.encoding_dim], dtype=np.float32)
        
        return encoding
    
    def encode_batch(self, values):
        """
        对一批值进行编码
        
        Args:
            values: shape (batch_size,) 的输入值
            
        Returns:
            encodings: shape (batch_size, encoding_dim) 的编码矩阵
        """
        return np.array([self.encode(v) for v in values], dtype=np.float32)


class FeatureEngineer:
    """
    特征工程编码器,使用手工设计的特征变换
    """
    def __init__(self, encoding_dim=5, max_value=10.0):
        """
        Args:
            encoding_dim: 编码输出的维度
            max_value: 输入值的预期最大值,用于归一化
        """
        self.encoding_dim = encoding_dim
        self.max_value = max_value
    
    def encode(self, value):
        """
        对单个标量值进行特征工程编码
        支持正负值（上楼为正，下楼为负）
        
        Args:
            value: 标量输入值(如楼梯高度，范围约为[-10, 10] cm)
            
        Returns:
            features: shape (encoding_dim,) 的特征向量
        """
        # 基础特征池（支持正负值）
        abs_value = np.abs(value)
        sign = np.sign(value) if value != 0 else 0.0
        
        feature_pool = [
            # 1. 原始值特征
            value,                                           # 原始值（保留符号）
            abs_value,                                       # 绝对值（高度大小）
            sign,                                            # 符号（上楼+1/下楼-1）
            
            # 2. 归一化特征
            value / self.max_value,                          # 归一化原始值
            abs_value / self.max_value,                      # 归一化绝对值
            
            # 3. 非线性变换（适用于正负值）
            value ** 2,                                      # 二次项（总是正值）
            value ** 3 / (self.max_value ** 2),             # 三次项（保留符号）
            np.sqrt(abs_value) * sign,                       # 开方（保留符号）
            
            # 4. 对数变换（处理负值）
            np.log(abs_value + 1) * sign,                    # 对数变换（保留符号）
            np.log1p(abs_value / self.max_value) * sign,    # 归一化对数
            
            # 5. 三角函数变换（周期性特征）
            np.sin(value / self.max_value * np.pi),         # 正弦变换
            np.cos(value / self.max_value * np.pi),         # 余弦变换
            np.sin(2 * value / self.max_value * np.pi),     # 高频正弦
            np.cos(2 * value / self.max_value * np.pi),     # 高频余弦
            
            # 6. 指数变换（捕捉非线性关系）
            np.exp(-abs_value / self.max_value) * sign,     # 指数衰减
            np.tanh(value / self.max_value),                 # 双曲正切（有界）
            
            # 7. 交互特征
            value * abs_value / (self.max_value ** 2),      # 值与绝对值交互
            sign * (abs_value / self.max_value) ** 2,       # 符号与归一化幅度平方
        ]
        
        # 根据需要的维度选择特征
        features = np.array(feature_pool[:self.encoding_dim], dtype=np.float32)
        
        # 如果特征池不够，循环使用（虽然不太可能发生）
        if len(features) < self.encoding_dim:
            # 添加更高阶的多项式特征
            for i in range(len(features), self.encoding_dim):
                power = i - len(feature_pool) + 4
                features = np.append(features, value ** power / (self.max_value ** (power - 1)))
        
        return features[:self.encoding_dim]
    
    def encode_batch(self, values):
        """
        对一批值进行编码
        
        Args:
            values: shape (batch_size,) 的输入值
            
        Returns:
            features: shape (batch_size, encoding_dim) 的特征矩阵
        """
        return np.array([self.encode(v) for v in values], dtype=np.float32)


def get_encoder(encoding_type='feature_engineering', encoding_dim=5, max_value=10.0):
    """
    工厂函数,根据类型创建编码器
    
    Args:
        encoding_type: 'feature_engineering' 或 'positional'
        encoding_dim: 编码输出维度
        max_value: 输入值的最大值
        
    Returns:
        encoder: FeatureEngineer 或 PositionalEncoder 实例
    """
    if encoding_type == 'feature_engineering':
        return FeatureEngineer(encoding_dim=encoding_dim, max_value=max_value)
    elif encoding_type == 'positional':
        return PositionalEncoder(encoding_dim=encoding_dim, max_value=max_value)
    else:
        raise ValueError(f"Unknown encoding type: {encoding_type}. "
                        f"Choose 'feature_engineering' or 'positional'.")


if __name__ == "__main__":
    """测试编码器"""
    print("=" * 60)
    print("测试特征工程编码器")
    print("=" * 60)
    fe_encoder = FeatureEngineer(encoding_dim=5, max_value=10.0)
    
    test_values = [4.0, 5.5, 7.0]
    for val in test_values:
        encoded = fe_encoder.encode(val)
        print(f"输入值: {val:.2f}")
        print(f"编码输出: {encoded}")
        print()
    
    print("=" * 60)
    print("测试位置编码器")
    print("=" * 60)
    pe_encoder = PositionalEncoder(encoding_dim=5, max_value=10.0)
    
    for val in test_values:
        encoded = pe_encoder.encode(val)
        print(f"输入值: {val:.2f}")
        print(f"编码输出: {encoded}")
        print()
    
    # 测试批量编码
    print("=" * 60)
    print("测试批量编码")
    print("=" * 60)
    batch_values = np.array([4.0, 5.0, 6.0, 7.0])
    
    fe_batch = fe_encoder.encode_batch(batch_values)
    print(f"特征工程批量编码 shape: {fe_batch.shape}")
    print(f"第一个样本: {fe_batch[0]}")
    
    pe_batch = pe_encoder.encode_batch(batch_values)
    print(f"位置编码批量编码 shape: {pe_batch.shape}")
    print(f"第一个样本: {pe_batch[0]}")
